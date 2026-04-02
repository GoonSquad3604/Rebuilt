package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.primaryLauncher.PrimaryLauncherIO;
import frc.robot.subsystems.shooter.primaryLauncher.PrimaryLauncherIOInputsAutoLogged;
import frc.robot.subsystems.shooter.secondaryLauncher.SecondaryLauncherIO;
import frc.robot.subsystems.shooter.secondaryLauncher.SecondaryLauncherIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  // private final HoodIO hoodIO;
  private final PrimaryLauncherIO primaryLauncherIO;
  private final SecondaryLauncherIO secondaryLauncherIO;
  private final TurretIO turretIO;

  private final SysIdRoutine primaryLauncherSysId;
  private final SysIdRoutine secondaryLauncherSysId;

  // private final SysIdRoutine turretSysId;

  // private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final PrimaryLauncherIOInputsAutoLogged primaryLauncherInputs =
      new PrimaryLauncherIOInputsAutoLogged();
  private final SecondaryLauncherIOInputsAutoLogged secondaryLauncherInputs =
      new SecondaryLauncherIOInputsAutoLogged();

  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  // private final Alert hoodMotorDisconnected;
  // private final Alert hoodEncoderDisconnected;
  private final Alert turretMotorDisconnected;
  private final Alert turretEncoderDisconnected;
  private final Alert primaryLauncherMotorDisconnected;
  private final Alert secondaryLauncherMotorDisconnected;

  private ShootingParameters shootingParameters;
  private ShootingParameters lastParameters;

  private boolean turretAtSetpoint = false;
  private boolean launchersAtSetpoint = false;

  // private double wantedHoodPosition;
  // private double dashboardHoodPosition;
  private double dashboardPrimaryLauncherVelocity;
  private double dashboardSecondaryLauncherVelocity;

  public enum ShooterWantedState {
    IDLE,
    TRACK_TARGET,
    SHOOT,
    EJECT,
    TEST_SHOOT,
    TRENCH,
    CLEAN
  }

  private enum CurrentState {
    IDLING,
    TRACKING_TARGET,
    SHOOTING_FORWARD,
    SHOOTING,
    EJECTING,
    TESTING_SHOOTING,
    ALIGNING_TO_TRENCH,
    CLEANING
  }

  private ShooterWantedState wantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(
      /*HoodIO hoodIO,*/ PrimaryLauncherIO primaryLauncherIO,
      SecondaryLauncherIO secondaryLauncherIO,
      TurretIO turretIO) {
    // this.hoodIO = hoodIO;
    this.primaryLauncherIO = primaryLauncherIO;
    this.secondaryLauncherIO = secondaryLauncherIO;
    this.turretIO = turretIO;

    // hoodMotorDisconnected = new Alert("Hood Motor Disconnected", Alert.AlertType.kWarning);
    // hoodEncoderDisconnected = new Alert("Hood Encoder Disconnected", Alert.AlertType.kWarning);
    turretMotorDisconnected = new Alert("Turret Motor Disconnected", Alert.AlertType.kWarning);
    turretEncoderDisconnected = new Alert("Turret Encoder Disconnected", Alert.AlertType.kWarning);
    primaryLauncherMotorDisconnected =
        new Alert("Primary Launcher Motor Disconnected", Alert.AlertType.kWarning);
    secondaryLauncherMotorDisconnected =
        new Alert("Secondary Launcher Motor Disconnected", Alert.AlertType.kWarning);

    primaryLauncherSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Shooter/PrimaryLauncher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> primaryLauncherIO.setOpenLoop(voltage.in(Volts)), null, this));

    secondaryLauncherSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Shooter/SecondaryLauncher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> secondaryLauncherIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // hoodIO.updateInputs(hoodInputs);
    primaryLauncherIO.updateInputs(primaryLauncherInputs);
    secondaryLauncherIO.updateInputs(secondaryLauncherInputs);
    turretIO.updateInputs(turretInputs);

    // Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/PrimaryLauncher", primaryLauncherInputs);
    Logger.processInputs("Subsystems/Shooter/SecondaryLauncher", secondaryLauncherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

    // Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);
    // Logger.recordOutput("Subsystems/Shooter/ReachedSetpoint", reachedSetpoints());

    if (ShotCalculator.getInstance().getParameters() != null) {
      if (lastParameters == null) {
        lastParameters = ShotCalculator.getInstance().getParameters();
      }
      shootingParameters = ShotCalculator.getInstance().getParameters();
      Logger.recordOutput("Subsystems/Shooter/WantedTurretAngle", shootingParameters.turretAngle());
    }

    CurrentState newState = handleStateTransitions();
    if (newState != currentState
        || lastParameters.turretAngle() != shootingParameters.turretAngle()
        // || lastParameters.hoodPose() != shootingParameters.hoodPose()
        || lastParameters.primaryFlywheelSpeed() != shootingParameters.primaryFlywheelSpeed()
        || lastParameters.secondaryFlywheelSpeed() != shootingParameters.secondaryFlywheelSpeed()) {

      currentState = newState;
      lastParameters = shootingParameters;
      Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
      applyStates();
    }

    SmartDashboard.putNumber(
        "TurretToHubMeters", RobotState.getInstance().getDistanceToHubMeters());

    // dashboardHoodPosition =
    //     SmartDashboard.getNumber("Hood Pose", ShooterConstants.HoodConstants.hoodMinPos);
    dashboardPrimaryLauncherVelocity = SmartDashboard.getNumber("PrimaryLauncher Velocity", 0);
    dashboardSecondaryLauncherVelocity = SmartDashboard.getNumber("SecondaryLauncher Velocity", 0);

    // SmartDashboard.putNumber("Hood Pose", dashboardHoodPosition);
    SmartDashboard.putNumber("PrimaryLauncher Velocity", dashboardPrimaryLauncherVelocity);
    SmartDashboard.putNumber("SecondaryLauncher Velocity", dashboardSecondaryLauncherVelocity);

    // hoodMotorDisconnected.set(!hoodInputs.motorConnected);
    // hoodEncoderDisconnected.set(!hoodInputs.encoderConnected);
    turretMotorDisconnected.set(!turretInputs.motorConnected);
    turretEncoderDisconnected.set(!turretInputs.encoderConnected);
    primaryLauncherMotorDisconnected.set(!primaryLauncherInputs.motorConnected);
    secondaryLauncherMotorDisconnected.set(!secondaryLauncherInputs.motorConnected);
  }

  public CurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> CurrentState.IDLING;
      case SHOOT -> RobotState.getInstance().isOverride()
              && RobotState.getInstance().getTarget() == ShooterTarget.FORWARD
          ? CurrentState.SHOOTING_FORWARD
          : CurrentState.SHOOTING;
      case EJECT -> CurrentState.EJECTING;
      case TEST_SHOOT -> CurrentState.TESTING_SHOOTING;
      case TRACK_TARGET -> CurrentState.TRACKING_TARGET;
      case TRENCH -> CurrentState.ALIGNING_TO_TRENCH;
      case CLEAN -> CurrentState.CLEANING;
    };
  }

  public void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case SHOOTING:
        shoot();
        break;
      case EJECTING:
        eject();
        break;
      case SHOOTING_FORWARD:
        shootForward();
        break;
      case TESTING_SHOOTING:
        testShoot();
        break;
      case TRACKING_TARGET:
        trackTarget();
        break;
      case ALIGNING_TO_TRENCH:
        trench();
        break;
      case CLEANING:
        clean();
        break;
    }
  }

  public boolean reachedSetpoints() {
    // if (shootingParameters != null
    //     && (wantedState == ShooterWantedState.SHOOT
    //         || wantedState == ShooterWantedState.TEST_SHOOT)) {

    if (shootingParameters == null) return false;

    // if override and shooting is forwards
    if (RobotState.getInstance().isOverride()
        && RobotState.getInstance().getTarget() == ShooterTarget.FORWARD) {

      launchersAtSetpoint =
          MathUtil.isNear(
                  ShooterConstants.PrimaryLauncherConstants.forwardVelocity,
                  primaryLauncherIO.getVelocity(),
                  ShooterConstants.PrimaryLauncherConstants.launcherAtSetpointTolerance)
              && MathUtil.isNear(
                  ShooterConstants.SecondaryLauncherConstants.forwardVelocity,
                  secondaryLauncherIO.getVelocity(),
                  ShooterConstants.SecondaryLauncherConstants.atSetpointTolerance);
    } else {
      // default shooting check systems for if at desired positions
      turretAtSetpoint =
          MathUtil.isNear(
              shootingParameters.turretAngle(),
              turretIO.getAngle(),
              ShooterConstants.TurretConstants.angleAtSetpointTolerance);
      launchersAtSetpoint =
          MathUtil.isNear(
                  shootingParameters.primaryFlywheelSpeed(),
                  primaryLauncherIO.getVelocity(),
                  ShooterConstants.PrimaryLauncherConstants.launcherAtSetpointTolerance)
              && MathUtil.isNear(
                  shootingParameters.secondaryFlywheelSpeed(),
                  secondaryLauncherIO.getVelocity(),
                  ShooterConstants.SecondaryLauncherConstants.atSetpointTolerance);
    }

    return turretAtSetpoint && launchersAtSetpoint && shootingParameters.validShootingLocation();
  }

  public void setWantedState(ShooterWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void idling() {
    // hoodIO.setPower(0);
    primaryLauncherIO.setPower(0);
    secondaryLauncherIO.setPower(0);
    turretIO.setPower(0);
  }

  private void shootForward() {
    turretIO.setPosition(ShooterConstants.TurretConstants.forwardPosition);
    // hoodIO.setPosition(ShooterConstants.HoodConstants.forwardPosition);
    primaryLauncherIO.setVelocity(ShooterConstants.PrimaryLauncherConstants.forwardVelocity);
    secondaryLauncherIO.setVelocity(ShooterConstants.SecondaryLauncherConstants.forwardVelocity);
  }

  private void shoot() {
    turretIO.setAngle(shootingParameters.turretAngle());
    // hoodIO.setPosition(shootingParameters.hoodPose());
    primaryLauncherIO.setVelocity(shootingParameters.primaryFlywheelSpeed());
    secondaryLauncherIO.setVelocity(shootingParameters.secondaryFlywheelSpeed());
  }

  private void eject() {
    turretIO.setPower(0.0);
    // turretIO.setAngle(shootingParameters.turretAngle());
    // hoodIO.setPosition(ShooterConstants.HoodConstants.hoodMinPos);
    primaryLauncherIO.setVelocity(shootingParameters.primaryFlywheelSpeed());
    secondaryLauncherIO.setVelocity(shootingParameters.primaryFlywheelSpeed());
  }

  private void testShoot() {
    primaryLauncherIO.setVelocity(dashboardPrimaryLauncherVelocity);
    secondaryLauncherIO.setVelocity(dashboardSecondaryLauncherVelocity);

    // hoodIO.setPosition(dashboardHoodPosition);
    turretIO.setAngle(shootingParameters.turretAngle());
  }

  private void trackTarget() {
    // shooterIsReady = false;
    turretIO.setAngle(shootingParameters.turretAngle());
    primaryLauncherIO.setPower(0);
    secondaryLauncherIO.setPower(0);
    // hoodIO.setPosition(shootingParameters.hoodPose());
  }

  private void trench() {
    turretIO.setAngle(shootingParameters.turretAngle());
    primaryLauncherIO.setPower(0.0);
    secondaryLauncherIO.setPower(0.0);

    // hoodIO.setPosition(ShooterConstants.HoodConstants.hoodMinPos);
  }

  private void clean() {
    turretIO.setPower(0.0);
    primaryLauncherIO.setPower(ShooterConstants.PrimaryLauncherConstants.cleanSpeed);
    secondaryLauncherIO.setPower(ShooterConstants.SecondaryLauncherConstants.cleanSpeed);

    // hoodIO.setPower(0.0);
  }

  // public void setBeganFiring(boolean newValue) {
  //   beganFiring = newValue;
  // }

  // testcontroller:
  public void setTurretPower(double power) {
    turretIO.setPower(power);
  }

  public void setTurretPos(double position) {
    turretIO.setPosition(position);
  }

  // public void setHoodPower(double power) {
  //   // hoodIO.setPower(power);
  // }

  // public void setHoodPosition(double position) {
  //   // hoodIO.setPosition(position);
  // }

  // public void setMagicHoodPosition(double position) {
  // hoodIO.setPositionMotionMagic(position);
  // }

  public void setPrimaryLauncherPower(double power) {
    primaryLauncherIO.setPower(power);
  }

  public void setPrimaryLauncherVelocity(double velocity) {
    primaryLauncherIO.setVelocity(velocity);
  }

  public void setSecondaryLauncherPower(double power) {
    secondaryLauncherIO.setPower(power);
  }

  public void setSecondaryLauncherVelocity(double velocity) {
    secondaryLauncherIO.setVelocity(velocity);
  }

  public void setDashboardSetpoints() {
    // hoodIO.setPosition(dashboardHoodPosition);
    primaryLauncherIO.setVelocity(dashboardPrimaryLauncherVelocity);
    secondaryLauncherIO.setVelocity(dashboardSecondaryLauncherVelocity);
  }

  public Command primaryLauncherSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> primaryLauncherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(primaryLauncherSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command primaryLauncherSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> primaryLauncherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(primaryLauncherSysId.dynamic(direction));
  }

  public Command secondaryLauncherSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> secondaryLauncherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(secondaryLauncherSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command secondaryLauncherSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> secondaryLauncherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(secondaryLauncherSysId.dynamic(direction));
  }

  // public Command turretSysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   return run(() -> turretIO.setTurretOpenLoop(0.0))
  //       .withTimeout(1.0)
  //       .andThen(turretSysId.quasistatic(direction));
  // }

  // /** Returns a command to run a dynamic test in the specified direction. */
  // public Command turretSysIdDynamic(SysIdRoutine.Direction direction) {
  //   return run(() -> turretIO.setTurretOpenLoop(0.0))
  //       .withTimeout(1.0)
  //       .andThen(turretSysId.dynamic(direction));
  // }
}
