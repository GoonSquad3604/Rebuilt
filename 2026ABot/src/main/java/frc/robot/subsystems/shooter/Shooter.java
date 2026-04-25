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
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final HoodIO hoodIO;
  private final LauncherIO launcherIO;
  private final TurretIO turretIO;

  private final SysIdRoutine launcherSysId;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final LauncherIOInputsAutoLogged launcherInputs = new LauncherIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  private final Alert hoodMotorDisconnected;
  private final Alert hoodEncoderDisconnected;
  private final Alert turretMotorDisconnected;
  private final Alert turretEncoderDisconnected;
  private final Alert launcherMotorDisconnected;

  private ShootingParameters shootingParameters;
  private ShootingParameters lastParameters;

  // private boolean beganFiring = false;

  private double dashboardHoodPosition;
  private double dashboardLauncherVelocity;

  public enum ShooterWantedState {
    IDLE,
    TRACK_TARGET,
    SHOOT,
    // PASS,
    EJECT,
    TEST_SHOOT,
    CLEAN,
  }

  private enum CurrentState {
    IDLING,
    TRACKING_TARGET,
    SHOOTING,
    PASSING,
    SHOOTING_FORWARD,
    EJECTING,
    TESTING_SHOOTING,
    CLEANING,
  }

  private ShooterWantedState wantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.launcherIO = launcherIO;
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;

    hoodMotorDisconnected = new Alert("Hood Motor Disconnected", Alert.AlertType.kWarning);
    hoodEncoderDisconnected = new Alert("Hood Encoder Disconnected", Alert.AlertType.kWarning);
    turretMotorDisconnected = new Alert("Turret Motor Disconnected", Alert.AlertType.kWarning);
    turretEncoderDisconnected = new Alert("Turret Encoder Disconnected", Alert.AlertType.kWarning);
    launcherMotorDisconnected = new Alert("Launcher Motor Disconnected", Alert.AlertType.kWarning);

    launcherSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Shooter/Launcher/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> launcherIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    launcherIO.updateInputs(launcherInputs);
    turretIO.updateInputs(turretInputs);

    Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

    if (ShotCalculator.getInstance().getParameters() != null) {
      if (lastParameters == null) {
        lastParameters = ShotCalculator.getInstance().getParameters();
      }
      shootingParameters = ShotCalculator.getInstance().getParameters();
      Logger.recordOutput("Subsystems/Shooter/WantedTurretAngle", shootingParameters.turretAngle());
      Logger.recordOutput(
          "Subsystems/Shooter/WantedHoodPosition", shootingParameters.hoodPosition());
    }

    CurrentState newState = handleStateTransitions();
    if (newState != currentState || lastParameters != shootingParameters) {

      currentState = newState;
      lastParameters = shootingParameters;
      Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
      applyStates();
    }

    SmartDashboard.putNumber(
        "TurretToHubMeters", RobotState.getInstance().getDistanceToHubMeters());

    dashboardHoodPosition =
        SmartDashboard.getNumber("Hood Pose", ShooterConstants.HoodConstants.hoodMinPos);
    dashboardLauncherVelocity = SmartDashboard.getNumber("Launcher Velocity", 0);

    SmartDashboard.putNumber("Hood Pose", dashboardHoodPosition);
    SmartDashboard.putNumber("Launcher Velocity", dashboardLauncherVelocity);

    Logger.recordOutput("Subsystems/Shooter/inDeadZone", !atValidShootingLocation());

    Logger.recordOutput("Subsystems/Shooter/launcherAtSetpoint", launcherAtSetpoint());
    Logger.recordOutput("Subsystems/Shooter/turretAtSetpoint", turretAtSetpoint());
    Logger.recordOutput("Subsystems/Shooter/hoodAtSetpoint", hoodAtSetpoint());

    hoodMotorDisconnected.set(!hoodInputs.motorConnected);
    hoodEncoderDisconnected.set(!hoodInputs.encoderConnected);
    turretMotorDisconnected.set(!turretInputs.motorConnected);
    turretEncoderDisconnected.set(!turretInputs.encoderConnected);
    launcherMotorDisconnected.set(!launcherInputs.motorConnected);
  }

  public CurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> CurrentState.IDLING;
      case SHOOT -> RobotState.getInstance().isOverride()
              && RobotState.getInstance().getTarget() == ShooterTarget.FORWARD
          ? CurrentState.SHOOTING_FORWARD
          : RobotState.getInstance().getTarget() != ShooterTarget.HUB
              ? CurrentState.PASSING
              : CurrentState.SHOOTING;
      case EJECT -> CurrentState.EJECTING;
      case TEST_SHOOT -> CurrentState.TESTING_SHOOTING;
      case TRACK_TARGET -> CurrentState.TRACKING_TARGET;
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
      case PASSING:
        pass();
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
      case CLEANING:
        clean();
        break;
    }
  }

  public boolean launcherAtSetpoint() {
    if (shootingParameters == null) return false;

    boolean launcherAtSetpoint = false;

    if (currentState == CurrentState.SHOOTING_FORWARD) {
      launcherAtSetpoint =
          MathUtil.isNear(
              ShooterConstants.LauncherConstants.forwardVelocity,
              launcherIO.getVelocity(),
              ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);
    } else if (currentState == CurrentState.TESTING_SHOOTING) {
      launcherAtSetpoint =
          MathUtil.isNear(
              dashboardLauncherVelocity,
              launcherIO.getVelocity(),
              ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);
    } else {
      if (currentState == CurrentState.PASSING) {
        launcherAtSetpoint =
            MathUtil.isNear(
                shootingParameters.passFlywheelVelocity(),
                launcherIO.getVelocity(),
                ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);
      } else {
        launcherAtSetpoint =
            MathUtil.isNear(
                shootingParameters.flywheelVelocity(),
                launcherIO.getVelocity(),
                ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);
      }
    }

    return launcherAtSetpoint;
  }

  public boolean turretAtSetpoint() {
    if (shootingParameters == null) return false;

    boolean turretAtSetpoint = false;

    if (currentState == CurrentState.SHOOTING_FORWARD) {
      turretAtSetpoint =
          MathUtil.isNear(
              ShooterConstants.TurretConstants.forwardAngle,
              turretIO.getAngle(),
              ShooterConstants.TurretConstants.atSetpointTolerance);
    } else {
      turretAtSetpoint =
          MathUtil.isNear(
              shootingParameters.turretAngle(),
              turretIO.getAngle(),
              ShooterConstants.TurretConstants.atSetpointTolerance);
    }

    if (shootingParameters.turretAngle() > ShooterConstants.TurretConstants.maxAnglePosition
        || shootingParameters.turretAngle() < ShooterConstants.TurretConstants.minAnglePosition) {
      return false;
    }

    return turretAtSetpoint;
  }

  public boolean hoodAtSetpoint() {
    if (shootingParameters == null) return false;

    boolean hoodAtSetpoint = false;

    if (currentState == CurrentState.SHOOTING_FORWARD) {
      hoodAtSetpoint =
          MathUtil.isNear(
              ShooterConstants.HoodConstants.forwardPosition,
              hoodIO.getPosition(),
              ShooterConstants.HoodConstants.hoodAtSetpointTolerance);
    } else {
      if (currentState == CurrentState.PASSING) {
        hoodAtSetpoint =
            MathUtil.isNear(
                ShooterConstants.HoodConstants.hoodMaxPos,
                hoodIO.getPosition(),
                ShooterConstants.HoodConstants.hoodAtSetpointTolerance);
      } else {
        hoodAtSetpoint =
            MathUtil.isNear(
                shootingParameters.hoodPosition(),
                hoodIO.getPosition(),
                ShooterConstants.HoodConstants.hoodAtSetpointTolerance);
      }
    }

    return hoodAtSetpoint;
  }

  public boolean atValidShootingLocation() {
    return shootingParameters.validShootingLocation();
  }

  public void setWantedState(ShooterWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void idling() {
    hoodIO.setPower(0);
    launcherIO.setPower(0);
    turretIO.setPower(0);
  }

  private void shootForward() {
    turretIO.setAngle(ShooterConstants.TurretConstants.forwardAngle);
    hoodIO.setPosition(ShooterConstants.HoodConstants.forwardPosition);
    launcherIO.setVelocity(ShooterConstants.LauncherConstants.forwardVelocity);
  }

  private void shoot() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPosition());
    launcherIO.setVelocity(shootingParameters.flywheelVelocity());
  }

  private void pass() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPosition());
    launcherIO.setVelocity(shootingParameters.passFlywheelVelocity());
  }

  private void eject() {
    turretIO.setPower(0.0);
    hoodIO.setPosition(ShooterConstants.HoodConstants.hoodMinPos);
    launcherIO.setVelocity(shootingParameters.flywheelVelocity());
  }

  private void testShoot() {
    launcherIO.setVelocity(dashboardLauncherVelocity);
    hoodIO.setPosition(dashboardHoodPosition);
    turretIO.setAngle(shootingParameters.turretAngle());
  }

  private void trackTarget() {
    turretIO.setAngle(shootingParameters.turretAngle());
    launcherIO.setPower(0.0);
    hoodIO.setPosition(ShooterConstants.HoodConstants.hoodMinPos);
  }

  private void clean() {
    turretIO.setPower(0.0);
    launcherIO.setPower(ShooterConstants.LauncherConstants.cleanSpeed);
    hoodIO.setPower(0.0);
  }

  // testcontroller:
  public void setTurretPower(double power) {
    turretIO.setPower(power);
  }

  public void setTurretPos(double position) {
    turretIO.setPosition(position);
  }

  public void setHoodPower(double power) {
    hoodIO.setPower(power);
  }

  public void setHoodPosition(double position) {
    hoodIO.setPosition(position);
  }

  public void setPrimaryLauncherPower(double power) {
    launcherIO.setPower(power);
  }

  public void setPrimaryLauncherVelocity(double velocity) {
    launcherIO.setVelocity(velocity);
  }

  public void setDashboardSetpoints() {
    hoodIO.setPosition(dashboardHoodPosition);
    launcherIO.setVelocity(dashboardLauncherVelocity);
  }

  public Command launcherSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> launcherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(launcherSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command launcherSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> launcherIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(launcherSysId.dynamic(direction));
  }
}
