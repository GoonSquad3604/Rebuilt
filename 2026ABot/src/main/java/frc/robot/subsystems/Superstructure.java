// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
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

  private boolean tracking = true;
  private WantedSuperState stateBeforeTrenchAlign;

  public enum WantedSuperState {
    STOPPED,
    INTAKE,
    STOW,
    SHOOT,
    INTAKE_AND_SHOOT,
    CLIMB,
    SET_UP_AUTO_CLIMB,
    CLIMB_IN_AUTO,
    DECLIMB,
    EJECT,
    TEST_SHOOT,
    STOP_TRACKING,
    ALIGN_TO_TRENCH
  }

  public enum CurrentSuperState {
    STOPPED,
    INTAKING,
    STOWING,
    SHOOTING,
    INTAKING_AND_SHOOTING,
    CLIMBING,
    SETTING_UP_AUTO_CLIMB,
    CLIMBING_IN_AUTO,
    DECLIMBING,
    EJECTING,
    TESTING_SHOOTING,
    QUITTING_TRACKING,
    ALIGNING_TO_TRENCH
  }

  private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
  private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;

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
    Logger.recordOutput("Subsystems/Superstructure/CurrentSuperState", currentSuperState);
    SmartDashboard.putBoolean("isOverride", RobotState.getInstance().isOverride());
    SmartDashboard.putString("manualTarget", RobotState.getInstance().getManualTarget().toString());

    SmartDashboard.putString("Alliance Shift Status", decideAllianceShiftInfo());

    // SmartDashboard.putBoolean("turret is tracking", tracking);

    currentSuperState = handleStateTransitions();
    applyStates();
  }

  private String decideAllianceShiftInfo() {
    double matchTime = Timer.getMatchTime();
    if (DriverStation.isAutonomous()) {
      return "AUTO: " + String.format("%.1f", matchTime);
    }
    if (matchTime > 130) {
      return "TRANSITION: " + String.format("%.1f", matchTime - 130);
    } else if (matchTime > 105) {
      return "SHIFT 1: " + String.format("%.1f", matchTime - 105);
    } else if (matchTime > 80) {
      return "SHIFT 2: " + String.format("%.1f", matchTime - 80);
    } else if (matchTime > 55) {
      return "SHIFT 3: " + String.format("%.1f", matchTime - 55);
    } else if (matchTime > 30) {
      return "SHIFT 4: " + String.format("%.1f", matchTime - 30);
    } else if (matchTime > 0) {
      return "ENDGAME: " + String.format("%.1f", matchTime);
    } else {
      return "DISABLED";
    }
  }

  public Command setWantedState(WantedSuperState state) {
    if (state == WantedSuperState.ALIGN_TO_TRENCH
        && stateBeforeTrenchAlign != WantedSuperState.ALIGN_TO_TRENCH) {
      stateBeforeTrenchAlign = wantedSuperState;
    }
    return Commands.runOnce(() -> wantedSuperState = state);
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    return switch (wantedSuperState) {
      case STOPPED -> CurrentSuperState.STOPPED;
      case INTAKE -> CurrentSuperState.INTAKING;
      case SHOOT -> CurrentSuperState.SHOOTING;
      case INTAKE_AND_SHOOT -> CurrentSuperState.INTAKING_AND_SHOOTING;
      case CLIMB -> CurrentSuperState.CLIMBING;
      case SET_UP_AUTO_CLIMB -> CurrentSuperState.SETTING_UP_AUTO_CLIMB;
      case CLIMB_IN_AUTO -> CurrentSuperState.CLIMBING_IN_AUTO;
      case DECLIMB -> CurrentSuperState.DECLIMBING;
      case STOW -> CurrentSuperState.STOWING;
      case EJECT -> CurrentSuperState.EJECTING;
      case TEST_SHOOT -> CurrentSuperState.TESTING_SHOOTING;
      case STOP_TRACKING -> CurrentSuperState.QUITTING_TRACKING;
      case ALIGN_TO_TRENCH -> CurrentSuperState.ALIGNING_TO_TRENCH;
    };
  }

  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        stopped();
        break;
      case INTAKING:
        intake();
        break;
      case SHOOTING:
        shoot();
        break;
      case INTAKING_AND_SHOOTING:
        intakeAndShoot();
        break;
      case SETTING_UP_AUTO_CLIMB:
        setUpAutoClimb();
        break;
      case CLIMBING_IN_AUTO:
        climbInAuto();
        break;
      case CLIMBING:
        climb();
        break;
      case DECLIMBING:
        declimb();
        break;
      case STOWING:
        stow();
        break;
      case EJECTING:
        eject();
        break;
      case TESTING_SHOOTING:
        testShoot();
        break;
      case QUITTING_TRACKING:
        quitTracking();
        break;
      case ALIGNING_TO_TRENCH:
        alignToTrench();
        break;
    }
  }

  private void stopped() {
    shooter.setBeganFiring(false);
    // climber.setWantedState(ClimberWantedState.IDLE);
    // hopper.setWantedState(HopperWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    if (tracking) {
      shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    } else {
      shooter.setWantedState(ShooterWantedState.IDLE);
    }
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void intake() {
    shooter.setBeganFiring(false);

    // climber.setWantedState(ClimberWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    if (tracking) {
      shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    } else {
      shooter.setWantedState(ShooterWantedState.IDLE);
    }
    spindexer.setWantedState(SpindexerWantedState.IDLE);

    if (climber.isStowed()) {
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.INTAKE);
      }
    }
  }

  private void stow() {
    // climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setBeganFiring(false);
    kicker.setWantedState(KickerWantedState.IDLE);
    if (tracking) {
      shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    } else {
      shooter.setWantedState(ShooterWantedState.IDLE);
    }
    spindexer.setWantedState(SpindexerWantedState.IDLE);

    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        wantedSuperState = WantedSuperState.STOPPED;
      }
    }
  }

  private void shoot() {
    // climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (climber.isStowed()) {
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.KICK);
      }
    }

    if (shooter.validShootingLocation() && shooter.reachedSetpoints()) {
      shooter.setBeganFiring(true);
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void intakeAndShoot() {
    // climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (climber.isStowed()) {
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.INTAKE);
      }
    }

    if (shooter.validShootingLocation() && shooter.reachedSetpoints()) {
      shooter.setBeganFiring(true);
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void climb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    shooter.setBeganFiring(false);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);

    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // ready to climb
        climber.setWantedState(ClimberWantedState.DEPLOY);
        if (climber.isDeployed() && climber.sensorsValid()) {
          // climb!!

        }
      }
    }
  }

  private void setUpAutoClimb() {
    shooter.setBeganFiring(false);
    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);

    // stow intake
    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      // stow hopper
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // everything is stowed, deploy climber
        if (!climber.isOuterDeployed()) {
          climber.setWantedState(ClimberWantedState.DEPLOY_OUTER);
        }
      }
    }
  }

  public void climbInAuto() {
    climber.setWantedState(ClimberWantedState.CLIMB_IN_AUTO);
    wantedSuperState = WantedSuperState.STOPPED;
  }

  private void declimb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    climber.setWantedState(ClimberWantedState.DECLIMB);
    wantedSuperState = WantedSuperState.STOPPED;
  }

  private void eject() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    // hopper.setWantedState(HopperWantedState.RESET);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
    kicker.setWantedState(KickerWantedState.REV);
    intake.setWantedState(IntakeWantedState.VOMIT);
  }

  private void testShoot() {
    shooter.setWantedState(ShooterWantedState.TEST_SHOOT);
    kicker.setWantedState(KickerWantedState.REV);
    spindexer.setWantedState(SpindexerWantedState.SPIN);

    if (climber.isStowed()) {
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.KICK);
      }
    }
  }

  private void quitTracking() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    tracking = !tracking;
    wantedSuperState = WantedSuperState.STOPPED;
  }

  private void alignToTrench() {
    shooter.setBeganFiring(false);
    shooter.setWantedState(ShooterWantedState.TRENCH);
  }

  public boolean climberDeployed() {
    return climber.isDeployed();
  }

  public WantedSuperState getStateBeforeTrenchAlign() {
    return stateBeforeTrenchAlign;
  }
}
