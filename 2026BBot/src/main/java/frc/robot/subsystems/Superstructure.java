// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberCurrentState;
import frc.robot.subsystems.climber.Climber.ClimberWantedState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerWantedState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeWantedState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterWantedState;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Climber climber;

  public enum WantedSuperState {
    STOPPED,
    INTAKE,
    VOMIT,
    INTAKE_AND_SHOOT,
    SHOOT,
    CLIMB,
    CLIMB_IN_AUTO,
    DECLIMB
  }

  public enum CurrentSuperState {
    STOPPED,
    INTAKING,
    VOMITING,
    INTAKING_AND_SHOOTING,
    SHOOTING,
    CLIMBING,
    CLIMBING_IN_AUTO,
    DECLIMBING
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, Climber climber) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.climber = climber;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Subsystems/Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Subsystems/Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Subsystems/Superstructure/PreviousSuperState", previousSuperState);

    currentSuperState = handleStateTransitions();
    applyStates();
  }

  public Command setWantedState(WantedSuperState state) {
    return Commands.runOnce(() -> wantedSuperState = state);
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case CLIMB:
        currentSuperState = CurrentSuperState.CLIMBING;
        break;
      case CLIMB_IN_AUTO:
        currentSuperState = CurrentSuperState.CLIMBING_IN_AUTO;
        break;
      case STOPPED:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case INTAKE_AND_SHOOT:
        currentSuperState = CurrentSuperState.INTAKING_AND_SHOOTING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case VOMIT:
        currentSuperState = CurrentSuperState.VOMITING;
        break;
      case DECLIMB:
        currentSuperState = CurrentSuperState.DECLIMBING;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case CLIMBING:
        climb();
        break;
      case STOPPED:
        stopped();
        break;
      case INTAKING:
        intake();
        break;
      case INTAKING_AND_SHOOTING:
        intakeAndShoot();
        break;
      case SHOOTING:
        shoot();
        break;
      case VOMITING:
        vomit();
        break;
      case DECLIMBING:
        declimb();
        break;
      case CLIMBING_IN_AUTO:
        climbInAuto();
        break;
    }
  }

  private void stopped() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);
    climber.setWantedState(ClimberWantedState.IDLE);
  }

  private void intake() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.INTAKE);
    indexer.setWantedState(IndexerWantedState.IDLE);
  }

  private void vomit() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.VOMIT);
    indexer.setWantedState(IndexerWantedState.VOMIT);
  }

  private void shoot() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    if (shooter.reachedSetpoint()) {
      intake.setWantedState(IntakeWantedState.INTAKE);
      indexer.setWantedState(IndexerWantedState.INDEX);
    } else {
      intake.setWantedState(IntakeWantedState.IDLE);
      indexer.setWantedState(IndexerWantedState.IDLE);
    }
  }

  private void intakeAndShoot() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    if (shooter.reachedSetpoint()) {
      intake.setWantedState(IntakeWantedState.INTAKE);
      indexer.setWantedState(IndexerWantedState.INDEX);
    } else {
      indexer.setWantedState(IndexerWantedState.IDLE);
    }
  }

  // full auto climb
  private void climb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);
    if (!climber.isAutoClimbing()) {
      climber.toggleIsAutoClimbing();
    }
  }

  // climb in auto
  private void climbInAuto() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);

    // if deployed and at position, set state to climb in auto, else continue to deploy
    if (climber.getCurrentState() == ClimberCurrentState.CLIMBING_LOW_RUNG_AUTO
        || (climber.getCurrentState() == ClimberCurrentState.DEPLOYED_OUTER
            && (RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)
                || RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)))) {
      climber.setWantedState(ClimberWantedState.CLIMB_LOW_RUNG_AUTO);
    } else {
      climber.setWantedState(ClimberWantedState.DEPLOY_OUTER);
    }
  }

  // descend
  private void declimb() {
    climber.setWantedState(ClimberWantedState.DECLIMB_LOW_RUNG);
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);
  }

  // public Command progressManualClimb() {
  //   return Commands.runOnce(() -> climber.updateClimberState(climber.decideNextClimbState()));
  // }

  // private void climbAndShoot() {
  //   shooter.setWantedState(ShooterWantedState.SHOOT);
  //   intake.setWantedState(IntakeWantedState.IDLE);
  //   indexer.setWantedState(IndexerWantedState.INDEX);
  // }
}
