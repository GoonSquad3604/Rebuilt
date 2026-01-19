// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private IndexerIO io;

  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer() {}

  public enum WantedState {
    OFF,
    INDEX,
    INDEX_TO_SHOOTER,
    VOMIT,
    AGITATE
  }

  private enum CurrentState {
    OFF,
    INDEXING,
    INDEXING_TO_SHOOTER,
    VOMITING,
    AGITATING
  }

  private WantedState wantedState = WantedState.OFF;

  private CurrentState currentState = CurrentState.OFF;

  private boolean isAllowedToCheckIfAlgaeHasEntered = false;
  private boolean hasAlgaeEntered = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    synchronized (inputs) {
      Logger.processInputs("Subsystems/Intake", inputs);

      currentState = handleStateTransition();
      applyState();
      Logger.recordOutput("Subsystems/Intake/SystemState", currentState);
      Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
    }
  }

  private CurrentState handleStateTransition() {
    return switch (wantedState) {
      case OFF:
        {
          yield CurrentState.OFF;
        }
      case INDEX:
        {
          yield CurrentState.INDEXING;
        }
      case INDEX_TO_SHOOTER:
        {
          yield CurrentState.INDEXING_TO_SHOOTER;
        }
      case VOMIT:
        {
          yield CurrentState.VOMITING;
        }
      case AGITATE:
        {
          yield CurrentState.AGITATING;
        }
    };
  }

  private void applyState() {
    double indexMotorVoltage = 0.0;
    double indexToShootMotorVoltage = 0.0;
    switch (currentState) {
      case OFF:
        break;
      case INDEXING:
        break;
      case INDEXING_TO_SHOOTER:
        break;
      case VOMITING:
        break;
      case AGITATING:
        break;
    }
    io.setIndexMotorVoltage(indexMotorVoltage);
    io.setIndexToShootMotorVoltage(indexToShootMotorVoltage);
  }
}
