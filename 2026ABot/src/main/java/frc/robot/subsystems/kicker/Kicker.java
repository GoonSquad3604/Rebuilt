package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Kicker extends SubsystemBase {

  private KickerIOPhoenix kickerIO = new KickerIOPhoenix();
  private KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  private SysIdRoutine sysID;

  public enum KickerWantedState {
    IDLE,
    REV
  }

  public enum KickerCurrentState {
    IDLING,
    REVVING,
  }

  private KickerCurrentState currentState = KickerCurrentState.IDLING;
  private KickerWantedState wantedState = KickerWantedState.IDLE;

  /** Creates a new Kicker. */
  public Kicker(KickerIOPhoenix io) {
    this.kickerIO = io;

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
      // applyStates();
    }

    Logger.recordOutput("Subsystems/Kicker/WantedState", wantedState);
  }

  public void setWantedState(KickerWantedState state) {
    wantedState = state;
  }

  private KickerCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> KickerCurrentState.IDLING;
      case REV -> KickerCurrentState.REVVING;
    };
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case REVVING:
        rev();
        break;
    }
  }

  private void idling() {
    kickerIO.setPower(0);
  }

  private void rev() {
    kickerIO.setPower(KickerConstants.shootingVelocity);
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
