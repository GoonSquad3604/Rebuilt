// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  // declare motors

  // io declaration
  private final IntakeIOPhoenix io = new IntakeIOPhoenix();

  public enum IntakeWantedState {
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
  private IntakeWantedState wantedState = IntakeWantedState.IDLE;

  /** Creates a new Intake. */
  public Intake(IntakeIOPhoenix io) {}

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

    switch (currentState) {
      case IDLING:
        stopIntake();
        break;

      case INTAKING:
        runIntake();
        break;

      case VOMITING:
        vomit();
        break;

      default:
        break;
    }
  }

  public void setWantedState(IntakeWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void runIntake() {
    io.setPower(IntakeConstants.intakeSpeed);
  }

  public void stopIntake() {
    io.setPower(0);
  }

  public void vomit() {
    io.setPower(IntakeConstants.vomitSpeed);
  }
}
