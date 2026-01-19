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
    DEPLOYED,
    STOWED // arm is stowed away / lift arm
  }

  private enum CurrentState {
    IDLING,
    INTAKING,
    VOMITING,
    DEPLOYED, // arm is down and ready
    DEPLOYING, // arm is dropping / entering desired pos
    STOWING // arm is returning to inside the robot
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

      case DEPLOYED:
        yield CurrentState.DEPLOYING;

      case STOWED:
        yield CurrentState.STOWING;
    };
  }

  private void applyStates() {
    double intakePower = 0;
    double newArmPos = 0;

    switch (currentState) {
      case IDLING:
        break;

      case INTAKING:
        intakePower = 0;
        break;

      case VOMITING:
        intakePower = 0;
        break;

      case DEPLOYED:
        if (wantedState == wantedState.INTAKE) intakePower = 0;
        break;

      case DEPLOYING:
        newArmPos = 0;
        break;

      case STOWING:
        newArmPos = 0;
        break;

      default:
        break;
    }

    io.setPower(intakePower);
    io.setArmPos(newArmPos);
  }
}
