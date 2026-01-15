// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public enum WantedState {
    DEPLOY_CLIMBER,
    STOWED,
    GO_TO_GROUND_FROM_LOW_RUNG,
    GO_TO_LOW_RUNG_FROM_GROUND,
    GO_TO_MID_RUNG_FROM_LOW_RUNG,
    GO_TO_HIGH_RUNG_FROM_MID_RUNG
  }

  public enum CurrentState {
    CLIMBER_DEPLOYED,
    DEPLOYING_CLIMBER,
    STOWING_CLIMBER,
    IS_STOWED,
    ON_GROUND,
    ON_LOW_RUNG,
    ON_MID_RUNG,
    ON_HIGH_RUNG,
    LOWERING_TO_GROUND,
    CLIMBING_TO_LOW_RUNG,
    CLIMBING_TO_MID_RUNG,
    CLIMBING_TO_HIGH_RUNG
  }

  private WantedState wantedState = WantedState.STOWED;
  private CurrentState currentState = CurrentState.IS_STOWED;

  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
