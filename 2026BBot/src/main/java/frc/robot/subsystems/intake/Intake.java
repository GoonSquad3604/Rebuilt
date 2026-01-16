// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  //declare motors

  //io declaration
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
    DEPLOYING,// arm is dropping / entering desired pos
    STOWING //arm is returning to inside the robot
  }

  private WantedState wantedState = WantedState.IDLE;

  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    
      //use wanted state to manage the transitions
      currentState = handleStateTransitions();
  }

  private CurrentState handleStateTransitions(){
    
    return switch(wantedState){

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

private void applyStates(){


  switch (currentState) {
    case IDLING:
      
      break;

    case INTAKING:
      io.setPower(0.0);
      break;
  
    case VOMITING:
      io.setPower(0);
      break;

    case DEPLOYED:
      if(wantedState == wantedState.INTAKE) io.setArmIntakeMotorPower(0);
      break;
  
    case DEPLOYING:
      io.setArmPos(0);
      break;

    case STOWING:
      io.setArmPos(0);
      break;

    default:
      break;
  }
}
}
