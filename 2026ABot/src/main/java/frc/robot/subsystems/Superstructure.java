// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberWantedState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.HopperWantedState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeWantedState;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.Kicker.KickerWantedState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterWantedState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerWantedState;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Drive drive;
  private final Climber climber;
  private final Hopper hopper;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Spindexer spindexer;

  public enum WantedSuperState {
    STOPPED,
    INTAKE,
    VOMIT,
    SHOOT,
    INTAKE_AND_SHOOT,
    CLIMB,
    CLIMB_AND_SHOOT,
    DECLIMB
  }

  public enum CurrentSuperState {
    STOPPED,
    INTAKING,
    VOMITING,
    SHOOTING,
    INTAKING_AND_SHOOTING,
    CLIMBING,
    CLIMBING_AND_SHOOTING,
    DECLIMBING
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
  private CurrentSuperState previousSuperState;

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive,
      Climber climber,
      Hopper hopper,
      Intake intake,
      Kicker kicker,
      Shooter shooter,
      Spindexer spindexer) {
    this.drive = drive;
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.kicker = kicker;
    this.shooter = shooter;
    this.spindexer = spindexer;
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
      case STOPPED:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case VOMIT:
        currentSuperState = CurrentSuperState.VOMITING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case INTAKE_AND_SHOOT:
        currentSuperState = CurrentSuperState.INTAKING_AND_SHOOTING;
        break;
      case CLIMB:
        currentSuperState = CurrentSuperState.CLIMBING;
        break;
      case CLIMB_AND_SHOOT:
        currentSuperState = CurrentSuperState.CLIMBING_AND_SHOOTING;
        break;
      case DECLIMB:
        currentSuperState = CurrentSuperState.DECLIMBING;
        break;
    }
    return currentSuperState;
  }

  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        stopped();
        break;
      case INTAKING:
        intake();
        break;
      case VOMITING:
        vomit();
        break;
      case SHOOTING:
        shoot();
        break;
      case INTAKING_AND_SHOOTING:
        intakeAndShoot();
        break;
      case CLIMBING:
        climb();
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
    // climber.setWantedState(ClimberWantedState.IDLE);
    // hopper.setWantedState(HopperWantedState.IDLE);
    // intake.setWantedState(IntakeWantedState.IDLE);
    // kicker.setWantedState(KickerWantedState.IDLE);
    // shooter.setWantedState(ShooterWantedState.IDLE);
    // spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void intake() {
    climber.setWantedState(ClimberWantedState.IDLE);
    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
    } else {
      intake.setWantedState(IntakeWantedState.INTAKE);
    }
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void vomit() {
    climber.setWantedState(ClimberWantedState.IDLE);
    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
    } else {
      intake.setWantedState(IntakeWantedState.VOMIT);
    }
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void shoot() {
    climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);
    if (shooter.reachedSetpoints() && kicker.atVelocity()) {
      intake.setWantedState(IntakeWantedState.KICK);
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      intake.setWantedState(IntakeWantedState.IDLE);
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void intakeAndShoot() {
    climber.setWantedState(ClimberWantedState.IDLE);
    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
    } else {
      intake.setWantedState(IntakeWantedState.INTAKE);
    }
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);
    if (shooter.reachedSetpoints() && kicker.atVelocity()) {
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void climb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void climbAndShoot() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    intake.setWantedState(IntakeWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
  }

  private void declimb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }
}
