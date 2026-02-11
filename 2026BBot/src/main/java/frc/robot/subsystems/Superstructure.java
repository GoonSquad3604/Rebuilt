// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberWantedState;
import frc.robot.subsystems.drive.Drive;
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
  // private final LED led;

  private enum TurretTarget {
    HUB,
    FORWARD,
    ZONE,
    CORRAL
  }

  public enum WantedSuperState {
    STOPPED,
    INTAKE,
    VOMIT,
    INTAKE_AND_SHOOT,
    SHOOT,
    CLIMB,
    CLIMB_AND_SHOOT,
    DECLIMB
  }

  public enum CurrentSuperState {
    STOPPED,
    INTAKING,
    VOMITING,
    INTAKING_AND_SHOOTING,
    SHOOTING,
    CLIMBING,
    CLIMBING_AND_SHOOTING,
    DECLIMBING
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  private TurretTarget turretTarget = TurretTarget.FORWARD;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, Climber climber) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.climber = climber;
    // this.led = led;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setWantedState(WantedSuperState state) {

    Logger.recordOutput("Superstructure/TurretTarget", turretTarget);

    Logger.recordOutput("Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Superstructure/CurrentSuperState", currentSuperState);
    Logger.recordOutput("Superstructure/PreviousSuperState", previousSuperState);

    currentSuperState = handleStateTransitions();
    applyStates();
  }

  private CurrentSuperState handleStateTransitions() {
    previousSuperState = currentSuperState;
    switch (wantedSuperState) {
      case CLIMB:
        currentSuperState = CurrentSuperState.CLIMBING;
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
      case CLIMB_AND_SHOOT:
        currentSuperState = CurrentSuperState.CLIMBING_AND_SHOOTING;
        break;
      case DECLIMB:
        currentSuperState = CurrentSuperState.DECLIMBING;
        break;
      default:
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
      case CLIMBING_AND_SHOOTING:
        climbAndShoot();
        break;
      case DECLIMBING:
        declimb();
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
    climber.setWantedState(ClimberWantedState.IDLE);
  }

  private void vomit() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.VOMIT);
    indexer.setWantedState(IndexerWantedState.VOMIT);
    climber.setWantedState(ClimberWantedState.IDLE);
  }

  private void shoot() {
    switch (turretTarget) {
      case CORRAL:
        shooter.setWantedState(ShooterWantedState.SHOOT_CORRAL);
        break;
      case FORWARD:
        shooter.setWantedState(ShooterWantedState.SHOOT_FORWARD);
        break;
      case HUB:
        shooter.setWantedState(ShooterWantedState.SHOOT_HUB);
        break;
      case ZONE:
        shooter.setWantedState(ShooterWantedState.SHOOT_ZONE);
        break;
    }
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.INDEX);
    climber.setWantedState(ClimberWantedState.IDLE);
  }

  private void intakeAndShoot() {
    switch (turretTarget) {
      case CORRAL:
        shooter.setWantedState(ShooterWantedState.SHOOT_CORRAL);
        break;
      case FORWARD:
        shooter.setWantedState(ShooterWantedState.SHOOT_FORWARD);
        break;
      case HUB:
        shooter.setWantedState(ShooterWantedState.SHOOT_HUB);
        break;
      case ZONE:
        shooter.setWantedState(ShooterWantedState.SHOOT_ZONE);
        break;
    }
    intake.setWantedState(IntakeWantedState.INTAKE);
    indexer.setWantedState(IndexerWantedState.INDEX);
    climber.setWantedState(ClimberWantedState.IDLE);
  }

  private void climb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);
    climber.setWantedState(ClimberWantedState.DEPLOY); // tbd
  }

  private void declimb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.IDLE);
    climber.setWantedState(ClimberWantedState.LOWER_TO_GROUND);
  }

  private void climbAndShoot() {
    switch (turretTarget) {
      case CORRAL:
        shooter.setWantedState(ShooterWantedState.SHOOT_CORRAL);
        break;
      case FORWARD:
        shooter.setWantedState(ShooterWantedState.SHOOT_FORWARD);
        break;
      case HUB:
        shooter.setWantedState(ShooterWantedState.SHOOT_HUB);
        break;
      case ZONE:
        shooter.setWantedState(ShooterWantedState.SHOOT_ZONE);
        break;
    }
    intake.setWantedState(IntakeWantedState.IDLE);
    indexer.setWantedState(IndexerWantedState.INDEX);
    climber.setWantedState(ClimberWantedState.DEPLOY); // tbd
  }
}
