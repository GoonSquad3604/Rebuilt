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

  private boolean beganFiring = false;
  private boolean readyToClimb = false;

  public enum WantedSuperState {
    STOPPED,
    TRACK,
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
    CLEAN
  }

  public enum CurrentSuperState {
    STOPPED,
    TRACKING,
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
    CLEANING
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
    return Commands.runOnce(() -> wantedSuperState = state);
  }

  public CurrentSuperState getCurrentSuperState() {
    return currentSuperState;
  }

  private CurrentSuperState handleStateTransitions() {
    return switch (wantedSuperState) {
      case STOPPED -> CurrentSuperState.STOPPED;
      case TRACK -> CurrentSuperState.TRACKING;
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
      case CLEAN -> CurrentSuperState.CLEANING;
    };
  }

  private void applyStates() {
    switch (currentSuperState) {
      case STOPPED:
        stopped();
        break;
      case TRACKING:
        track();
        break;
      case INTAKING:
        intake();
        break;
      case SHOOTING:
        shoot();
        break;
      case STOWING:
        stow();
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
      case EJECTING:
        eject();
        break;
      case TESTING_SHOOTING:
        testShoot();
        break;
      case CLEANING:
        clean();
        break;
    }
  }

  private void stopped() {
    beganFiring = false;
    climber.setWantedState(ClimberWantedState.IDLE);
    hopper.setWantedState(HopperWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void track() {
    beganFiring = false;
    // climber.setWantedState(ClimberWantedState.IDLE);
    // hopper.setWantedState(HopperWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
  }

  private void intake() {
    beganFiring = false;

    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.TRACK_TARGET);
    spindexer.setWantedState(SpindexerWantedState.IDLE);

    if (climber.isStowed()) {
      climber.setWantedState(ClimberWantedState.IDLE);
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.INTAKE);
      }
    } else {
      climber.setWantedState(ClimberWantedState.STOW);
    }
  }

  private void stow() {
    beganFiring = false;
    kicker.setWantedState(KickerWantedState.REV);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
    shooter.setWantedState(ShooterWantedState.EJECT);

    if (hopper.isStowed()) {
      wantedSuperState = WantedSuperState.STOPPED;
      return;
    }

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
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (climber.isStowed()) {
      climber.setWantedState(ClimberWantedState.IDLE);
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.KICK);
      }
    } else {
      climber.setWantedState(ClimberWantedState.STOW);
    }

    if ((shooter.launcherAtSetpoint() || beganFiring)
        && shooter.turretAtSetpoint()
        && shooter.atValidShootingLocation()) {
      beganFiring = true;
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void intakeAndShoot() {
    shooter.setWantedState(ShooterWantedState.SHOOT);
    kicker.setWantedState(KickerWantedState.REV);

    if (climber.isStowed()) {
      climber.setWantedState(ClimberWantedState.IDLE);
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.INTAKE);
      }
    } else {
      climber.setWantedState(ClimberWantedState.STOW);
    }

    if ((shooter.launcherAtSetpoint() || beganFiring)
        && shooter.turretAtSetpoint()
        && shooter.atValidShootingLocation()) {
      beganFiring = true;
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }
  }

  private void climb() {
    shooter.setWantedState(ShooterWantedState.IDLE);
    beganFiring = false;
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
        if ((climber.sensorsValid() && climber.isDeployed() && readyToClimb)
            || climber.beganAutoClimbing()) {
          climber.setWantedState(ClimberWantedState.CLIMB);
        }
      }
    }
  }

  private void setUpAutoClimb() {
    beganFiring = false;

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
    // wantedSuperState = WantedSuperState.STOPPED;
  }

  private void declimb() {
    climber.setWantedState(ClimberWantedState.DECLIMB);
    hopper.setWantedState(HopperWantedState.IDLE);
    intake.setWantedState(IntakeWantedState.IDLE);
    kicker.setWantedState(KickerWantedState.IDLE);
    shooter.setWantedState(ShooterWantedState.IDLE);
    spindexer.setWantedState(SpindexerWantedState.IDLE);
    wantedSuperState = WantedSuperState.TRACK;
  }

  private void eject() {
    hopper.setWantedState(HopperWantedState.DEPLOY);
    intake.setWantedState(IntakeWantedState.VOMIT);
    kicker.setWantedState(KickerWantedState.REV);
    shooter.setWantedState(ShooterWantedState.SHOOT);
    spindexer.setWantedState(SpindexerWantedState.SPIN);
  }

  private void testShoot() {
    shooter.setWantedState(ShooterWantedState.TEST_SHOOT);
    kicker.setWantedState(KickerWantedState.TEST);

    if ((shooter.launcherAtSetpoint() || beganFiring)
        && shooter.turretAtSetpoint()
        && shooter.atValidShootingLocation()) {
      beganFiring = true;
      spindexer.setWantedState(SpindexerWantedState.SPIN);
    } else {
      spindexer.setWantedState(SpindexerWantedState.IDLE);
    }

    if (climber.isStowed()) {
      if (!hopper.isDeployed()) {
        hopper.setWantedState(HopperWantedState.DEPLOY);
        intake.setWantedState(IntakeWantedState.STOW);
      } else {
        intake.setWantedState(IntakeWantedState.KICK);
      }
    }
  }

  private void clean() {
    shooter.setWantedState(ShooterWantedState.CLEAN);
    kicker.setWantedState(KickerWantedState.CLEAN);
    spindexer.setWantedState(SpindexerWantedState.CLEAN);
    if (!hopper.isDeployed()) {
      hopper.setWantedState(HopperWantedState.DEPLOY);
      intake.setWantedState(IntakeWantedState.STOW);
    } else {
      intake.setWantedState(IntakeWantedState.CLEAN);
    }
  }

  // helper methods

  public Command toggleReadyToClimb() {
    return Commands.runOnce(() -> readyToClimb = !readyToClimb);
  }

  public boolean climberDeployed() {
    return climber.isDeployed();
  }
}
