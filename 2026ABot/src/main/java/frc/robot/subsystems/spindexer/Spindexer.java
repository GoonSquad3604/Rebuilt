package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Spindexer extends SubsystemBase {

  private SpindexerIOPhoenix spindexerIO = new SpindexerIOPhoenix();
  private SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();

  private SysIdRoutine sysID;

  public enum SpindexerWantedState {
    IDLE,
    SPIN
  }

  private enum SpindexerCurrentState {
    IDLING,
    SPINNING
  }

  private SpindexerCurrentState currentState = SpindexerCurrentState.IDLING;
  private SpindexerWantedState wantedState = SpindexerWantedState.IDLE;

  /** Creates a new Hopper. */
  public Spindexer(SpindexerIOPhoenix io) {
    this.spindexerIO = io;
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
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Spindexer/CurrentState", currentState);
      applyStates();
    }

    Logger.recordOutput("Subsystems/Spindexer/WantedState", wantedState);
  }

  public void setWantedState(SpindexerWantedState state) {
    wantedState = state;
  }

  private SpindexerCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> SpindexerCurrentState.IDLING;
      case SPIN -> SpindexerCurrentState.SPINNING;
    };
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case SPINNING:
        spin();
        break;
    }
  }

  private void idling() {
    spindexerIO.setPower(0);
  }

  private void spin() {
    spindexerIO.setVelocity(SpindexerConstants.spinVelocity);
  }

  // testing only, remove later:
  public void setPower(double power) {
    spindexerIO.setPower(power);
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
