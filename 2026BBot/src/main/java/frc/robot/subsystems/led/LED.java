package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

  public enum AnimationType {
    STROBE_HUB,
    STROBE_CORRAL,
    STROBE_FORWARD,
    STROBE_ZONE,
    IDLE,
    RAINBOW
  }

  public enum WantedState {
    DISPLAY_OFF,
    DISPLAY_IDLE,
    DISPLAY_INTAKE_FORWARD,
    DISPLAY_INTAKE_HUB,
    DISPLAY_INTAKE_ZONE,
    DISPLAY_INTAKE_CORRAL,
    DISPLAY_FORWARD,
    DISPLAY_HUB,
    DISPLAY_ZONE,
    DISPLAY_CORRAL,
    DISPLAY_CLIMB
  }

  private enum CurrentState {
    DISPLAYING_OFF,
    DISPLAYING_IDLE,
    DISPLAYING_INTAKE_FORWARD,
    DISPLAYING_INTAKE_HUB,
    DISPLAYING_INTAKE_ZONE,
    DISPLAYING_INTAKE_CORRAL,
    DISPLAYING_FORWARD,
    DISPLAYING_HUB,
    DISPLAYING_ZONE,
    DISPLAYING_CORRAL,
    DISPLAYING_CLIMB
  }

  private WantedState wantedAction = WantedState.DISPLAY_OFF;
  private final LEDIO ledIO;

  private CurrentState getStateTransition() {
    return switch (wantedAction) {
      case DISPLAY_CLIMB -> CurrentState.DISPLAYING_CLIMB;
      case DISPLAY_CORRAL -> CurrentState.DISPLAYING_CORRAL;
      case DISPLAY_FORWARD -> CurrentState.DISPLAYING_FORWARD;
      case DISPLAY_HUB -> CurrentState.DISPLAYING_HUB;
      case DISPLAY_IDLE -> CurrentState.DISPLAYING_IDLE;
      case DISPLAY_INTAKE_CORRAL -> CurrentState.DISPLAYING_INTAKE_CORRAL;
      case DISPLAY_INTAKE_FORWARD -> CurrentState.DISPLAYING_INTAKE_FORWARD;
      case DISPLAY_INTAKE_HUB -> CurrentState.DISPLAYING_INTAKE_HUB;
      case DISPLAY_INTAKE_ZONE -> CurrentState.DISPLAYING_INTAKE_ZONE;
      case DISPLAY_OFF -> CurrentState.DISPLAYING_OFF;
      case DISPLAY_ZONE -> CurrentState.DISPLAYING_ZONE;
    };
  }

  public void setWantedAction(WantedState wantedAction) {
    this.wantedAction = wantedAction;
  }

  public LED(LEDIO ledIO) {
    this.ledIO = ledIO;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Subsystems/LED/WantedState", wantedAction);

    switch (getStateTransition()) {
      case DISPLAYING_OFF:
        ledIO.clearAnimation();
        ledIO.setLEDs(LEDConstants.kBlack);
        break;
      case DISPLAYING_IDLE:
        ledIO.setAnimation(AnimationType.IDLE);
        break;
      case DISPLAYING_CORRAL:
        ledIO.setLEDs(LEDConstants.kYellow);
        break;
      case DISPLAYING_FORWARD:
        ledIO.setLEDs(LEDConstants.kWhite);
        break;
      case DISPLAYING_HUB:
        ledIO.setLEDs(LEDConstants.kRed);
        break;
      case DISPLAYING_ZONE:
        ledIO.setLEDs(LEDConstants.kOrange);
        break;
      case DISPLAYING_INTAKE_CORRAL:
        ledIO.setAnimation(AnimationType.STROBE_CORRAL);
        break;
      case DISPLAYING_INTAKE_FORWARD:
        ledIO.setAnimation(AnimationType.STROBE_FORWARD);
        break;
      case DISPLAYING_INTAKE_HUB:
        ledIO.setAnimation(AnimationType.STROBE_HUB);
        break;
      case DISPLAYING_INTAKE_ZONE:
        ledIO.setAnimation(AnimationType.STROBE_ZONE);
        break;
      case DISPLAYING_CLIMB:
        ledIO.setAnimation(AnimationType.RAINBOW);
        break;
    }
  }
}
