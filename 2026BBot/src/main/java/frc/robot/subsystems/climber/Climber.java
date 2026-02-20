// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIOPhoenix io;

  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public enum ClimberWantedState {
    IDLE,
    DEPLOY,
    STOWED,
    LOWER_TO_GROUND,
    GO_TO_LOW_RUNG_FROM_GROUND,
    GO_TO_MID_RUNG_FROM_LOW_RUNG
    // GO_TO_HIGH_RUNG_FROM_MID_RUNG
  }

  public enum CurrentState {
    IDLING,

    DEPLOYED,
    DEPLOYING,

    STOWING,
    STOWED,

    ON_GROUND,
    ON_LOW_RUNG,
    ON_MID_RUNG,
    // ON_HIGH_RUNG,

    LOWERING_TO_GROUND,
    CLIMBING_TO_LOW_RUNG,
    CLIMBING_TO_MID_RUNG
    // CLIMBING_TO_HIGH_RUNG
  }

  private ClimberWantedState wantedState = ClimberWantedState.STOWED;
  private CurrentState currentState = CurrentState.STOWED;

  public Climber(ClimberIOPhoenix io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.processInputs("Subsystems/Climber", inputs);

    currentState = handleStateTransitions();
    applyStates();
  }

  public void setWantedState(ClimberWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private CurrentState handleStateTransitions() {

    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLING;

      case DEPLOY:
        yield CurrentState.DEPLOYING;

      case STOWED:
        yield CurrentState.STOWING;

      case LOWER_TO_GROUND:
        yield CurrentState.LOWERING_TO_GROUND;

      case GO_TO_LOW_RUNG_FROM_GROUND:
        yield CurrentState.CLIMBING_TO_LOW_RUNG;

      case GO_TO_MID_RUNG_FROM_LOW_RUNG:
        yield CurrentState.CLIMBING_TO_MID_RUNG;
    };
  }

  private void applyStates() {

    switch (currentState) {
      case IDLING:
        stopped();
        break;

      case DEPLOYING:
        deploy();
        break;

      case STOWING:
        stow();
        break;

      case LOWERING_TO_GROUND:
        decesend();
        break;

      case CLIMBING_TO_LOW_RUNG:
        climbLowRung();
        break;

      case CLIMBING_TO_MID_RUNG:
        climbMidRung();
        break;

      default:
        break;
    }
  }

  private void stopped() {
    io.setPowerLowRung(0);
    io.setPowerMidRung(0);
  }

  private void deploy() {}

  private void stow() {}

  private void decesend() {}

  private void climbLowRung() {}

  private void climbMidRung() {}

  public void setPowerLowRung(double power) {
    io.setPowerLowRung(power);
  }

  public void setPowerMidRung(double power) {
    io.setPowerMidRung(power);
  }
}
