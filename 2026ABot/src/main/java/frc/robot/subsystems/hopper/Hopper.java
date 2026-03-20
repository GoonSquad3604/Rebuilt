package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

  private HopperIOPhoenix hopperIO = new HopperIOPhoenix();
  private HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();

  private final Alert hopperMotorDisconnected;
  private final Alert stowedDetectorDisconnected;

  private SysIdRoutine sysID;

  private boolean canMove = true;

  public enum HopperWantedState {
    IDLE,
    STOW,
    FORCE_STOW,
    RESET,
    DEPLOY
  }

  private enum HopperCurrentState {
    IDLING,
    STOWING_FAST,
    STOWING_SLOW,
    FORCEFULLY_STOWING,
    STOWED,
    RESETTING,
    DEPLOYING,
  }

  private HopperCurrentState currentState = HopperCurrentState.IDLING;
  private HopperWantedState wantedState = HopperWantedState.IDLE;

  /** Creates a new Hopper. */
  public Hopper(HopperIOPhoenix io) {
    this.hopperIO = io;

    hopperMotorDisconnected = new Alert("Hopper Motor Disconnected", Alert.AlertType.kWarning);
    stowedDetectorDisconnected =
        new Alert("Stowed Detector Disconnected", Alert.AlertType.kWarning);

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
    // Logger.recordOutput("Subsystems/Hopper/isDeployed", isDeployed());
    // Logger.recordOutput("Subsystems/Hopper/isStowed", isStowed());

    HopperCurrentState newState = handleStateTransitions();
    if (newState != currentState
        || (wantedState == HopperWantedState.STOW && !isStowed())
        || (wantedState == HopperWantedState.FORCE_STOW && !isStowed())) {
      currentState = newState;
      // Logger.recordOutput("Subsystems/Hopper/CurrentState", currentState);
      applyStates();
    }

    // Logger.recordOutput("Subsystems/Hopper/WantedState", wantedState);

    hopperMotorDisconnected.set(!hopperInputs.motorConnected);
    stowedDetectorDisconnected.set(!hopperInputs.stowedDetectorConnected);
  }

  public void setWantedState(HopperWantedState state) {
    wantedState = state;
  }

  private HopperCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> HopperCurrentState.IDLING;
      case DEPLOY -> HopperCurrentState.DEPLOYING;
      case STOW -> isStowed()
          ? HopperCurrentState.IDLING
          : hopperIO.getPosition() > HopperConstants.stowTargetPosition
              ? HopperCurrentState.STOWING_SLOW
              : HopperCurrentState.STOWING_FAST;

      case FORCE_STOW -> isStowed()
          ? HopperCurrentState.IDLING
          : HopperCurrentState.FORCEFULLY_STOWING;
      case RESET -> HopperCurrentState.RESETTING;
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
      case STOWED:
        stowed();
        break;
      case FORCEFULLY_STOWING:
        forceStow();
        break;
      case RESETTING:
        reset();
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

  private void forceStow() {
    hopperIO.setPower(HopperConstants.forceStowPower);
  }

  private void idling() {
    hopperIO.setPower(0);
  }

  private void stowed() {
    hopperIO.setPower(0);
  }

  private void reset() {
    hopperIO.setPower(-.2);
  }

  public boolean isDeployed() {
    return MathUtil.isNear(
        HopperConstants.extendedPos, hopperIO.getPosition(), HopperConstants.atSetpointTolerance);
  }

  public boolean isStowed() {
    return hopperIO.stowedDetectorTriggered();
  }

  public void zeroEncoder() {
    hopperIO.resetPosition();
  }

  public void setDeployed() {
    hopperIO.setEncoderPosition(HopperConstants.extendedPos);
  }

  public void setHopperCanMove(boolean canMove) {
    this.canMove = canMove;
  }

  public boolean hopperCanMove() {
    return canMove;
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
