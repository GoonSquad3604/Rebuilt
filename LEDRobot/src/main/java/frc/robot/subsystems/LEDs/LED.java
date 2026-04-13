// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private LEDConstants constants = new LEDConstants();
  private LEDIOCANdle io = new LEDIOCANdle();
  private final LEDIOInputsAutoLogged LEDInputs = new LEDIOInputsAutoLogged();

  public enum WantedState {
    INTAKE,
    SHOOT,
    INTAKE_AND_SHOOT,
    CLIMB,
    ALIGN,
    IDLE,
    OFF
  }

  public enum CurrentState {
    INTAKING,
    SHOOTING,
    INTAKE_AND_SHOOTING,
    CLIMBING,
    ALIGNING,
    IDLING,
    OFF
  }

  public WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  @Override
  public void periodic() {
    io.updateInputs(LEDInputs);

    Logger.processInputs("Subsystems/LEDs", LEDInputs);

    CurrentState newState = handleStateTransitions();

    if (newState != currentState) {
      io.turnOff();
      currentState = newState;
      Logger.recordOutput("Subsystems/LEDs/CurrentState", currentState);
      applyStates();
    }
  }

  public CurrentState handleStateTransitions() {
    switch (wantedState) {
      case INTAKE:
        return CurrentState.INTAKING;
      case SHOOT:
        return CurrentState.SHOOTING;
      case INTAKE_AND_SHOOT:
        return CurrentState.INTAKE_AND_SHOOTING;
      case CLIMB:
        return CurrentState.CLIMBING;
      case ALIGN:
        return CurrentState.ALIGNING;
      case OFF:
        return CurrentState.OFF;
      default:
        return CurrentState.IDLING;
    }
  }

  public void setWantedState(WantedState state) {
    wantedState = state;
  }

  private void applyStates() {
    switch (currentState) {
      case INTAKING:
        io.setStrobe(constants.red);
        break;

      case SHOOTING:
        io.setFire();
        break;

      case INTAKE_AND_SHOOTING:
        io.setFire();
        break;

      case CLIMBING:
        io.setRainbow();
        break;

      case ALIGNING:
        io.setStrobe(constants.orange);
        break;

      case IDLING:
        io.setColor(constants.purple);
        break;

      case OFF:
        io.turnOff();
        break;
    }
  }
}
