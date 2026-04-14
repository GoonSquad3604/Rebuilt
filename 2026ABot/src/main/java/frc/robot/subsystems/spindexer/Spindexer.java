package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Spindexer extends SubsystemBase {

  private SpindexerIOPhoenix spindexerIO = new SpindexerIOPhoenix();
  private SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();

  private final Alert spindexerMotorDisconnected;

  private SysIdRoutine sysID;

  private boolean beganUnjamming;
  private double timeBeganUnjamming;

  public enum SpindexerWantedState {
    IDLE,
    SPIN,
    CLEAN,
    UNJAM
  }

  private enum SpindexerCurrentState {
    IDLING,
    SPINNING,
    CLEANING,
    UNJAMMING
  }

  private SpindexerCurrentState currentState = SpindexerCurrentState.IDLING;
  private SpindexerWantedState wantedState = SpindexerWantedState.IDLE;

  /** Creates a new Hopper. */
  public Spindexer(SpindexerIOPhoenix io) {
    this.spindexerIO = io;

    spindexerMotorDisconnected =
        new Alert("Spindexer Motor Disconnected", Alert.AlertType.kWarning);

    sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Subsystems/Spindexer/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> spindexerIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Subsystems/Spindexer", spindexerInputs);

    SpindexerCurrentState newState = handleStateTransitions();

    if (newState != currentState || currentState == SpindexerCurrentState.UNJAMMING) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Spindexer/CurrentState", currentState);
      applyStates();
    }

    // Logger.recordOutput("Subsystems/Spindexer/WantedState", wantedState);

    spindexerMotorDisconnected.set(!spindexerInputs.motorConnected);
  }

  public void setWantedState(SpindexerWantedState state) {
    wantedState = state;
  }

  private SpindexerCurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return SpindexerCurrentState.IDLING;
      case SPIN:
        return continueUnjamming()
            ? SpindexerCurrentState.UNJAMMING
            : SpindexerCurrentState.SPINNING;
      case CLEAN:
        return SpindexerCurrentState.CLEANING;
      case UNJAM:
        beganUnjamming = true;
        timeBeganUnjamming = Timer.getFPGATimestamp();
        wantedState = SpindexerWantedState.SPIN;
        return SpindexerCurrentState.UNJAMMING;
    }
    return SpindexerCurrentState.IDLING;
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case SPINNING:
        spin();
        break;
      case CLEANING:
        clean();
        break;
      case UNJAMMING:
        unjam();
        break;
    }
  }

  private void idling() {
    spindexerIO.setPower(0);
  }

  private void spin() {
    spindexerIO.setVelocity(SpindexerConstants.spinVelocity);
  }

  private void clean() {
    spindexerIO.setPower(SpindexerConstants.cleanSpeed);
  }

  private void unjam() {
    spindexerIO.setVelocity(SpindexerConstants.unjamVelocity);
  }

  // checks for if jammed
  private boolean continueUnjamming() {
    if (!beganUnjamming) return false;
    double newTimestamp = Timer.getFPGATimestamp();
    boolean shouldStopUnjamming =
        timeBeganUnjamming < newTimestamp - SpindexerConstants.unjamDuration;
    if (shouldStopUnjamming) {
      beganUnjamming = false;
    }
    return shouldStopUnjamming;
  }

  // testing only, remove later:
  public void setPower(double power) {
    spindexerIO.setPower(power);
  }

  public void setVelocity(double velocity) {
    spindexerIO.setVelocity(velocity);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> spindexerIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(sysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> spindexerIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(sysID.dynamic(direction));
  }
}
