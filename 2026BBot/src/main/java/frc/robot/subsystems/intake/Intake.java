// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // declare motors

  // io declaration
  IntakeIOPhoenix io = new IntakeIOPhoenix();

  public enum WantedState {
    IDLE,
    INTAKE,
    VOMIT,
  }

  private enum CurrentState {
    IDLING,
    INTAKING,
    VOMITING,
  }

  private CurrentState currentState = CurrentState.IDLING;
  private WantedState wantedState = WantedState.IDLE;

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {

    // use states to do stuff
    currentState = handleStateTransitions();
    applyStates();
  }

  private CurrentState handleStateTransitions() {

    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLING;

      case INTAKE:
        yield CurrentState.INTAKING;

      case VOMIT:
        yield CurrentState.VOMITING;
    };
  }

  private void applyStates() {
    double intakePower = 0;

    switch (currentState) {
      case IDLING:
        break;

      case INTAKING:
        intakePower = 0;
        break;

      case VOMITING:
        intakePower = 0;
        break;

      default:
        break;
    }

    io.setPower(intakePower);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
