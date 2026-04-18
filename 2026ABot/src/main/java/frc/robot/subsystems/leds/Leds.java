package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {

  private LedsIO ledsIO = new LedsIOCANdle();
  private LedsIOInputsAutoLogged ledInputs = new LedsIOInputsAutoLogged();
  private final Alert CANdleDisconnected;

  private double lastColorSwitch = 0.0;
  private boolean isRed = false;

  public enum LedsWantedState {
    IDLE,
    STOW,
    INTAKE,
    SHOOT,
    INTAKE_AND_SHOOT,
    CLIMB,
    TRACK,
    POLICE,
    CLEAN,
    EJECT,
    UNJAM
  }

  public enum CurrentState {
    IDLING,
    STOWING,
    INTAKING,
    SHOOTING,
    INTAKE_AND_SHOOTING,
    CLIMBING,
    TRACKING,
    POLICING,
    CLEANING,
    EJECTING,
    UNJAMMING
  }

  private LedsWantedState wantedState = LedsWantedState.IDLE;
  private CurrentState currentState;

  /** Creates a new Leds. */
  public Leds(LedsIOCANdle ledsIO) {
    this.ledsIO = ledsIO;

    CANdleDisconnected = new Alert("CANdle Disconnected", Alert.AlertType.kWarning);
  }

  @Override
  public void periodic() {

    ledsIO.updateInputs(ledInputs);
    Logger.processInputs("Subsystems/Leds", ledInputs);

    CurrentState newState = handleStateTransitions();

    if (newState != currentState || currentState == CurrentState.POLICING) {

      if (currentState == CurrentState.POLICING) {
        double lastTimestamp = Timer.getFPGATimestamp();
        if (lastTimestamp > lastColorSwitch + LedConstants.colorSwitchInterval) {
          lastColorSwitch = Timer.getFPGATimestamp();
          isRed = !isRed;
        }
      }
      ledsIO.turnOff();
      currentState = newState;
      Logger.recordOutput("Subsystems/LEDs/CurrentState", currentState);
      applyStates();
    }
    CANdleDisconnected.set(!ledInputs.CANdleConnected);
  }

  public CurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> CurrentState.IDLING;
      case STOW -> CurrentState.STOWING;
      case INTAKE -> CurrentState.INTAKING;
      case SHOOT -> CurrentState.SHOOTING;
      case INTAKE_AND_SHOOT -> CurrentState.INTAKE_AND_SHOOTING;
      case CLIMB -> CurrentState.CLIMBING;
      case TRACK -> CurrentState.TRACKING;
      case EJECT -> CurrentState.EJECTING;
      case CLEAN -> CurrentState.CLEANING;
      case POLICE -> CurrentState.POLICING;
      case UNJAM -> CurrentState.UNJAMMING;
    };
  }

  public void setWantedState(LedsWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        ledsIO.setFlow(LedConstants.purple);
        break;
      case STOWING:
        ledsIO.setStrobe(LedConstants.blue);
        break;
      case INTAKING:
        ledsIO.setStrobe(LedConstants.red);
        break;
      case SHOOTING:
        ledsIO.setFire();
        break;
      case INTAKE_AND_SHOOTING:
        ledsIO.setStrobe(LedConstants.red, LedConstants.orange);
        break;
      case CLIMBING:
        ledsIO.setRainbow();
        break;
      case TRACKING:
        ledsIO.setColor(LedConstants.purple);
        break;
      case EJECTING:
        ledsIO.setStrobe(LedConstants.red);
        break;
      case CLEANING:
        ledsIO.setColor(LedConstants.green);
        break;
      case POLICING:
        if (isRed) {
          ledsIO.setColor(LedConstants.red);
        } else {
          ledsIO.setColor(LedConstants.blue);
        }
        break;
      case UNJAMMING:
        ledsIO.setColor(LedConstants.green);
        break;
    }
  }
}
