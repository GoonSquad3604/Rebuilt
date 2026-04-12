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
  // private final Alert joeCoderDisconnected;
  private final Alert stowedDetectorDisconnected;

  private SysIdRoutine sysID;

  private boolean recentlyStowed = true;
  private boolean wasDeployed = false;

  public enum HopperWantedState {
    IDLE,
    STOW,
    DEPLOY
  }

  private enum HopperCurrentState {
    IDLING,
    HOLDING_IN,
    STOWING_PID,
    STOWING_POWER,
    STOWED,
    DEPLOYING,
    DEPLOYED
  }

  private HopperCurrentState currentState = HopperCurrentState.IDLING;
  private HopperWantedState wantedState = HopperWantedState.IDLE;

  /** Creates a new Hopper. */
  public Hopper(HopperIOPhoenix io) {
    this.hopperIO = io;

    hopperMotorDisconnected = new Alert("Hopper Motor Disconnected", Alert.AlertType.kWarning);
    // joeCoderDisconnected = new Alert("JoeCoder Disconnected", Alert.AlertType.kWarning);
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
    if (newState != currentState || (wantedState == HopperWantedState.STOW && !isStowed())) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Hopper/CurrentState", currentState);
      applyStates();
    }

    // Logger.recordOutput("Subsystems/Hopper/WantedState", wantedState);

    // recently stowed logic
    if (recentlyStowed && isStowed()) {
      recentlyStowed = false;
    }

    // alerts
    hopperMotorDisconnected.set(!hopperInputs.motorConnected);
    // joeCoderDisconnected.set(!hopperInputs.joeCoderConnected);
    stowedDetectorDisconnected.set(!hopperInputs.stowedDetectorConnected);
  }

  public void setWantedState(HopperWantedState state) {
    wantedState = state;
  }

  private HopperCurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return HopperCurrentState.IDLING;
      case DEPLOY:
        return HopperCurrentState.DEPLOYING;
      case STOW:
        if (isStowed()) {
          // wantedState = HopperWantedState.IDLE;
          return HopperCurrentState.HOLDING_IN;
        } else {
          return HopperCurrentState.STOWING_POWER;
          // if (hopperIO.getPosition() < HopperConstants.stowTargetPosition) {
          //   return HopperCurrentState.STOWING_POWER;
          // } else {
          //   return HopperCurrentState.STOWING_PID;
          // }
        }
    }
    return HopperCurrentState.IDLING;
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case HOLDING_IN:
        holdIn();
        break;
      case DEPLOYING:
        deploy();
        break;
      case DEPLOYED:
        deployed();
        break;
      case STOWING_PID:
        stowFast();
        break;
      case STOWING_POWER:
        stowSlow();
        break;
      case STOWED:
        stowed();
        break;
    }
  }

  private void deploy() {
    hopperIO.setPosition(HopperConstants.extendedPos);
  }

  private void holdIn() {
    // hopperIO.setPower(HopperConstants.holdHopperInPower);
    hopperIO.setPower(0.0);
  }

  private void deployed() {
    // wasDeployed = true;
    // hopperIO.setPower(0.0);
  }

  private void stowFast() {
    wasDeployed = false;
    hopperIO.setPosition(HopperConstants.stowTargetPosition);
  }

  private void stowSlow() {
    wasDeployed = false;
    hopperIO.setPower(HopperConstants.slowStowingPower);
  }

  private void idling() {
    hopperIO.setPower(0);
  }

  private void stowed() {
    wasDeployed = false;
    hopperIO.setPower(0);
  }

  public boolean isDeployed() {
    return MathUtil.isNear(
            HopperConstants.extendedPos,
            hopperIO.getPosition(),
            HopperConstants.atSetpointTolerance)
        || wasDeployed;
  }

  public boolean isStowed() {
    // if (!recentlyStowed) {
    //   recentlyStowed = true;
    hopperIO.setEncoderPosition(0.0);
    // }
    return hopperIO.stowedDetectorTriggered();
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
