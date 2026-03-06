package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private HopperIOPhoenix hopperIO = new HopperIOPhoenix();
  private HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  private SysIdRoutine sysID;

  private boolean deployed = false;

  public enum HopperWantedState {
    IDLE,
    STOW,
    DEPLOY
  }

  private enum HopperCurrentState {
    IDLING,
    STOWING_FAST,
    STOWING_SLOW,
    DEPLOYING,
  }

  private HopperCurrentState currentState = HopperCurrentState.IDLING;
  private HopperWantedState wantedState = HopperWantedState.IDLE;

  /** Creates a new Hopper. */
  public Hopper(HopperIOPhoenix io) {
    this.hopperIO = io;
    sysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Subsystems/Hopper/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> hopperIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    hopperIO.updateInputs(hopperInputs);
    Logger.processInputs("Subsystems/Hopper", hopperInputs);

    HopperCurrentState newState = handleStateTransitions();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Hopper/CurrentState", currentState);
      applyStates();
    } else {
      if (currentState == HopperCurrentState.STOWING_SLOW && hopperIO.stowedDetectorTriggered()) {
        this.setWantedState(HopperWantedState.IDLE);
        hopperIO.resetPosition();
      }
    }

    if (MathUtil.isNear(HopperConstants.extendedPos, hopperIO.getPosition(), 5)) {
      deployed = true;
    } else {
      deployed = false;
    }

    Logger.recordOutput("Subsystems/Hopper/WantedState", wantedState);
  }

  public void setWantedState(HopperWantedState state) {
    wantedState = state;
  }

  private HopperCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> HopperCurrentState.IDLING;
      case DEPLOY -> HopperCurrentState.DEPLOYING;
      case STOW -> MathUtil.isNear(
              HopperConstants.stowTargetPosition,
              hopperIO.getPosition(),
              HopperConstants.atSetpointTolerance)
          ? HopperCurrentState.STOWING_SLOW
          : HopperCurrentState.STOWING_FAST;
    };
  }

  private void applyStates() {
    switch (currentState) {
      case DEPLOYING:
        deploy();
        break;
      case IDLING:
        idling();
        break;
      case STOWING_FAST:
        stowFast();
        break;
      case STOWING_SLOW:
        stowSlow();
        break;
    }
  }

  private void deploy() {
    hopperIO.setPosition(HopperConstants.extendedPos);
  }

  private void stowFast() {
    hopperIO.setPosition(HopperConstants.stowTargetPosition);
  }

  private void stowSlow() {
    hopperIO.setPower(HopperConstants.slowStowingPower);
  }

  private void idling() {
    hopperIO.setPower(0);
  }

  public boolean isDeployed() {
    return deployed;
  }

  // testing only, remove later:
  public void setPower(double power) {
    hopperIO.setPower(power);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> hopperIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(sysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> hopperIO.setOpenLoop(0.0)).withTimeout(1.0).andThen(sysID.dynamic(direction));
  }
}
