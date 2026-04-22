package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Kicker extends SubsystemBase {

  private KickerIOPhoenix kickerIO = new KickerIOPhoenix();
  private KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  private final Alert kickerMotorDisconnected;

  private SysIdRoutine sysID;

  private double dashboardKickerVelocity;

  private boolean isJammed = false;
  // private boolean beganJamming = false;
  // private double timeBeganJamming;

  private boolean beganUnjamming;
  private double timeBeganUnjamming;

  public enum KickerWantedState {
    IDLE,
    REV,
    TEST,
    CLEAN,
    EJECT,
    UNJAM
  }

  public enum KickerCurrentState {
    IDLING,
    REVVING,
    TESTING,
    CLEANING,
    EJECTING,
    UNJAMMING
  }

  private KickerCurrentState currentState = KickerCurrentState.IDLING;
  private KickerWantedState wantedState = KickerWantedState.IDLE;

  /** Creates a new Kicker. */
  public Kicker(KickerIOPhoenix io) {
    this.kickerIO = io;

    kickerMotorDisconnected = new Alert("Kicker Motor Disconnected", Alert.AlertType.kWarning);

    sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Subsystems/Kicker/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> kickerIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Subsystems/Kicker", kickerInputs);

    KickerCurrentState newState = handleStateTransitions();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Kicker/CurrentState", currentState);
      applyStates();
    }

    // if (currentState == KickerCurrentState.REVVING) {
    //   updateisJammed();
    // }

    dashboardKickerVelocity =
        SmartDashboard.getNumber("Kicker Velocity", KickerConstants.shootingVelocity);
    SmartDashboard.putNumber("Kicker Velocity", dashboardKickerVelocity);

    // Logger.recordOutput("Subsystems/Kicker/WantedState", wantedState);

    kickerMotorDisconnected.set(!kickerInputs.motorConnected);
  }

  public void setWantedState(KickerWantedState state) {
    this.wantedState = state;
  }

  private KickerCurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return KickerCurrentState.IDLING;
        // case REV: return KickerCurrentState.REVVING;
      case REV:
        return continueUnjamming() ? KickerCurrentState.UNJAMMING : KickerCurrentState.REVVING;
      case TEST:
        return KickerCurrentState.TESTING;
      case CLEAN:
        return KickerCurrentState.CLEANING;
      case EJECT:
        return KickerCurrentState.EJECTING;
      case UNJAM:
        // beganUnjamming = true;
        // timeBeganUnjamming = Timer.getFPGATimestamp();
        // wantedState = KickerWantedState.REV;
        return KickerCurrentState.UNJAMMING;
    }
    return KickerCurrentState.IDLING;
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case REVVING:
        rev();
        break;
      case TESTING:
        test();
        break;
      case CLEANING:
        clean();
        break;
      case EJECTING:
        eject();
        break;
      case UNJAMMING:
        unjam();
        break;
    }
  }

  private void idling() {
    kickerIO.setPower(0);
  }

  private void rev() {
    kickerIO.setVelocity(KickerConstants.shootingVelocity);
  }

  private void test() {
    kickerIO.setVelocity(dashboardKickerVelocity);
  }

  private void clean() {
    kickerIO.setPower(KickerConstants.cleanSpeed);
  }

  private void eject() {
    kickerIO.setPower(.9);
  }

  private void unjam() {
    kickerIO.setVelocity(KickerConstants.unjamVelocity);
  }

  private boolean continueUnjamming() {
    if (!beganUnjamming) return false;
    double newTimestamp = Timer.getFPGATimestamp();
    boolean shouldStopUnjamming = timeBeganUnjamming < newTimestamp - KickerConstants.unjamDuration;
    if (shouldStopUnjamming) {
      beganUnjamming = false;
    }
    return shouldStopUnjamming;
  }

  // private void updateisJammed() {
  //   if (kickerIO.getVelocity() > KickerConstants.minJammedVelocity) {
  //     if (!beganJamming) {
  //       timeBeganJamming = Timer.getFPGATimestamp();
  //     }
  //     beganJamming = true;
  //     double newTimestamp = Timer.getFPGATimestamp();
  //     if (timeBeganJamming < newTimestamp - KickerConstants.jamCheckTimeDuration) {
  //       isJammed = true;
  //     }
  //   } else {
  //     beganJamming = false;
  //     isJammed = false;
  //   }
  // }

  public boolean isJammed() {
    return isJammed;
  }

  public boolean atVelocity() {
    return MathUtil.isNear(
        KickerConstants.shootingVelocity,
        kickerIO.getVelocity(),
        KickerConstants.shootingVelocityTolerance);
  }

  // testing only, remove later:
  public void setPower(double power) {
    kickerIO.setPower(power);
  }

  public void setVelocity(double velocity) {
    kickerIO.setVelocity(velocity);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> kickerIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(sysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> kickerIO.setOpenLoop(0.0)).withTimeout(1.0).andThen(sysID.dynamic(direction));
  }
}
