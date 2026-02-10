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

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public enum IndexerWantedState {
    IDLE,
    INDEX,
    VOMIT,
  }

  private enum CurrentState {
    IDLING,
    INDEXING,
    VOMITING,
  }

  private IndexerWantedState wantedState = IndexerWantedState.IDLE;

  private CurrentState currentState = CurrentState.IDLING;


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
      case IDLE:
        {
          yield CurrentState.IDLING;
        }
      case INDEX:
        {
          yield CurrentState.INDEXING;
        }
      case VOMIT:
        {
          yield CurrentState.VOMITING;
        }
    };
  }

  private void applyState() {
    double indexMotorVoltage = 0.0;
    switch (currentState) {
      case IDLING:
        stopped();
        break;
      case INDEXING:
        indexing();
        break;
      case VOMITING:
        vommiting();
        break;
    }
    io.setIndexMotorVoltage(indexMotorVoltage);
  }

  public void setWantedState(IndexerWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void stopped(){
    io.setIndexPower(0);
  }

  private void indexing(){
    io.setIndexPower(0.5);
  }

  private void vommiting(){
    io.setIndexPower(-0.5);
  }

}
