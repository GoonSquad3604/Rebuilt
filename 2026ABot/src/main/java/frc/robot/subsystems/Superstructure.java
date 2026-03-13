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
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberWantedState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
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
import java.util.Set;
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
    STOW,
    SHOOT,
    INTAKE_AND_SHOOT,
    CLIMB_LEFT,
    CLIMB_RIGHT,
    DECLIMB,
    EJECT,
    TEST_SHOOT
  }

  public enum CurrentSuperState {
    STOPPED,
    INTAKING,
    STOWING,
    SHOOTING,
    INTAKING_AND_SHOOTING,
    CLIMBING_LEFT,
    CLIMBING_RIGHT,
    CLIMBING_LEFT_L1,
    CLIMBING_RIGHT_L1,
    DECLIMBING,
    EJECTING,
    TESTING_SHOOTING
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
    // Logger.recordOutput("Subsystems/Superstructure/WantedSuperState", wantedSuperState);
    Logger.recordOutput("Subsystems/Superstructure/CurrentSuperState", currentSuperState);
    // Logger.recordOutput("Subsystems/Superstructure/PreviousSuperState", previousSuperState);
    // Logger.recordOutput("RobotState/Override", RobotState.getInstance().isOverride());
    // SmartDashboard.putBoolean("isOverride", RobotState.getInstance().isOverride());
    // SmartDashboard.putString("manualTarget",
    // RobotState.getInstance().getManualTarget().toString());

    SmartDashboard.putString("Alliance Shift Status", decideAllianceShiftInfo());

    // Logger.recordOutput("RobotState/ManualTarget", RobotState.getInstance().getManualTarget());

    currentSuperState = handleStateTransitions();
    applyStates();

    // new Trigger(() -> climber.isAutoClimbing()).onTrue(new AutomatedClimb(climber));
  }

  private String decideAllianceShiftInfo() {
    double matchTime = Timer.getMatchTime();
    // Boolean weWonAuto =
    //     DriverStation.getGameSpecificMessage()
    //         .equalsIgnoreCase(DriverStation.getAlliance().get().toString());
    if (DriverStation.isAutonomous()) {
      return "AUTO: " + String.format("%.1f", matchTime);
    }
    // if (!weWonAuto.equals(null)) {}

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
    return Commands.runOnce(() -> wantedSuperState = state);
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    switch (wantedSuperState) {
      case STOPPED:
        currentSuperState = CurrentSuperState.STOPPED;
        break;
      case INTAKE:
        currentSuperState = CurrentSuperState.INTAKING;
        break;
      case SHOOT:
        currentSuperState = CurrentSuperState.SHOOTING;
        break;
      case INTAKE_AND_SHOOT:
        currentSuperState = CurrentSuperState.INTAKING_AND_SHOOTING;
        break;
      case CLIMB_LEFT:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.CLIMBING_LEFT_L1
                : CurrentSuperState.CLIMBING_LEFT;
        break;
      case CLIMB_RIGHT:
        currentSuperState =
            DriverStation.isAutonomous()
                ? CurrentSuperState.CLIMBING_RIGHT_L1
                : CurrentSuperState.CLIMBING_RIGHT;
        break;
      case DECLIMB:
        currentSuperState = CurrentSuperState.DECLIMBING;
        break;
      case STOW:
        currentSuperState = CurrentSuperState.STOWING;
        break;
      case EJECT:
        currentSuperState = CurrentSuperState.EJECTING;
        break;
      case TEST_SHOOT:
        currentSuperState = CurrentSuperState.TESTING_SHOOTING;
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
      case SHOOTING:
        shoot();
        break;
      case INTAKING_AND_SHOOTING:
        intakeAndShoot();
        break;
      case CLIMBING_LEFT:
        climbLeft();
        break;
      case CLIMBING_RIGHT:
        climbRight();
        break;
      case CLIMBING_LEFT_L1:
        climbLeftAuto();
        break;
      case CLIMBING_RIGHT_L1:
        climbRightAuto();
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
    }
  }

  private void stopped() {
    climber.setWantedState(ClimberWantedState.IDLE);
    // hopper.setWantedState(HopperWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void intake() {
    climber.setWantedState(ClimberWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    spindexer.setWantedState(SpindexerWantedState.IDLE);

    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
      intake.setWantedState(IntakeWantedState.STOW);
    } else {
      intake.setWantedState(IntakeWantedState.INTAKE);
    }
  }

  private void stow() {
    climber.setWantedState(ClimberWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
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
    climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
      intake.setWantedState(IntakeWantedState.STOW);
    } else {
      intake.setWantedState(IntakeWantedState.KICK);
    }

    // if (shooter.reachedSetpoints() && kicker.atVelocity()) {
    spindexer.setWantedState(SpindexerWantedState.SPIN);
    // } else {
    //   spindexer.setWantedState(SpindexerWantedState.IDLE);
    // }
  }

  private void intakeAndShoot() {
    climber.setWantedState(ClimberWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
      intake.setWantedState(IntakeWantedState.STOW);
    } else {
      intake.setWantedState(IntakeWantedState.INTAKE);
    }

    // if (shooter.reachedSetpoints() && kicker.atVelocity()) {
    spindexer.setWantedState(SpindexerWantedState.SPIN);
    // } else {
    //   spindexer.setWantedState(SpindexerWantedState.IDLE);
    // }
  }

  private void climbLeft() {
    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // ready to climb
        climber.setWantedState(ClimberWantedState.DEPLOY);
        if (RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)) {
          // climb
          climber.setWantedState(ClimberWantedState.CLIMB);
        } else {
          // drive to climb pose
          Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
              .andThen(DriveCommands.alignToPose(drive, DriveConstants.leftClimbPos));
        }
      }
    }

    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
  }

  private void climbRight() {
    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // ready to climb
        climber.setWantedState(ClimberWantedState.DEPLOY);
        if (RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)) {
          // climb
          climber.setWantedState(ClimberWantedState.CLIMB);
        } else {
          // drive to climb pose
          Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
              .andThen(DriveCommands.alignToPose(drive, DriveConstants.rightClimbPos));
        }
      }
    }

    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
  }

  private void climbLeftAuto() {
    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // ready to climb
        climber.setWantedState(ClimberWantedState.DEPLOY);
        if (RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)) {
          // climb
          climber.setWantedState(ClimberWantedState.CLIMB_L1);
        } else {
          // drive to climb pose
          Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
              .andThen(DriveCommands.alignToPose(drive, DriveConstants.leftClimbPos));
        }
      }
    }

    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
  }

  private void climbRightAuto() {
    if (!intake.isStowed()) {
      intake.setWantedState(IntakeWantedState.STOW);
      hopper.setWantedState(HopperWantedState.IDLE);
    } else {
      if (!hopper.isStowed()) {
        hopper.setWantedState(HopperWantedState.STOW);
      } else {
        // ready to climb
        climber.setWantedState(ClimberWantedState.DEPLOY);
        if (RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)) {
          // climb
          climber.setWantedState(ClimberWantedState.CLIMB_L1);
        } else {
          // drive to climb pose
          Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
              .andThen(DriveCommands.alignToPose(drive, DriveConstants.rightClimbPos));
        }
      }
    }

    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
  }

  private void declimb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    // intake.setWantedState(IntakeWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void eject() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
    kicker.setWantedState(KickerWantedState.REV);
    intake.setWantedState(IntakeWantedState.VOMIT);
  }

  private void testShoot() {
    shooter.setWantedState(ShooterWantedState.TEST_SHOOT);
    kicker.setWantedState(KickerWantedState.REV);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
  }
}
