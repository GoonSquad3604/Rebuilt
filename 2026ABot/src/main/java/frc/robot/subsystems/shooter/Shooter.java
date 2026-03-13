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
  private final SysIdRoutine turretSysId;

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

  private boolean turretAtSetpoint = false;
  private boolean hoodAtSetpoint = false;
  private boolean launcherAtSetpoint = false;

  private double wantedHoodPosition;
  private double dashboardHoodPosition;
  private double dashboardLauncherVelocity;

  public enum ShooterWantedState {
    IDLE,
    TRACK_TARGET,
    SHOOT,
    TEST_SHOOT
  }

  private enum CurrentState {
    IDLING,
    TRACKING_TARGET,
    SHOOTING_FORWARD,
    SHOOTING,
    TESTING_SHOOTING
  }

  private ShooterWantedState wantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
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
                (voltage) -> launcherIO.setLauncherOpenLoop(voltage.in(Volts)), null, this));

    turretSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Subsystems/Shooter/Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> turretIO.setTurretOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(hoodInputs);
    launcherIO.updateInputs(launcherInputs);
    turretIO.updateInputs(turretInputs);

    // Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

    // Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);

    // Logger.recordOutput("Subsystems/Shooter/TurretAtSetpoint", turretAtSetpoint);
    // Logger.recordOutput("Subsystems/Shooter/HoodAtSetpoint", hoodAtSetpoint);
    // Logger.recordOutput("Subsystems/Shooter/LauncherAtSetpoint", launcherAtSetpoint);
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
        || lastParameters.hoodPose() != shootingParameters.hoodPose()
        || lastParameters.flywheelSpeed() != shootingParameters.flywheelSpeed()) {

      currentState = newState;
      lastParameters = shootingParameters;
      // Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
      applyStates();
    }

    SmartDashboard.putNumber(
        "TurretToHubMeters", RobotState.getInstance().getDistanceToHubMeters());
    // SmartDashboard.putNumber(
    //     "TurretToHubInches", RobotState.getInstance().getDistanceToHubInches());

    // dashboardHoodPosition =
    //     SmartDashboard.getNumber("Hood Pose", ShooterConstants.HoodConstants.hoodMinPos);
    dashboardLauncherVelocity = SmartDashboard.getNumber("Launcher Velocity", 0);

    // SmartDashboard.putNumber("Hood Pose", dashboardHoodPosition);
    SmartDashboard.putNumber("Launcher Velocity", dashboardLauncherVelocity);

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
          : CurrentState.SHOOTING;
      case TEST_SHOOT -> CurrentState.TESTING_SHOOTING;
      case TRACK_TARGET -> CurrentState.TRACKING_TARGET;
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
      case SHOOTING_FORWARD:
        shootForward();
        break;
      case TESTING_SHOOTING:
        testShoot();
        break;
      case TRACKING_TARGET:
        trackTarget();
        break;
    }
  }

  public boolean reachedSetpoints() {
    if (shootingParameters != null && wantedState == ShooterWantedState.SHOOT) {

      // check systems for if at desired positions
      turretAtSetpoint =
          MathUtil.isNear(
              shootingParameters.turretAngle(),
              turretIO.getAngle(),
              ShooterConstants.TurretConstants.angleAtSetpointTolerance);
      hoodAtSetpoint = MathUtil.isNear(shootingParameters.hoodPose(), hoodIO.getPosition(), 0.2);
      launcherAtSetpoint =
          MathUtil.isNear(
              shootingParameters.flywheelSpeed(),
              launcherIO.getVelocity(),
              ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);

      if (RobotState.getInstance().isOverride()
          && RobotState.getInstance().getTarget() == ShooterTarget.FORWARD) {
        // if overriding for forward shot
        return MathUtil.isNear(
            ShooterConstants.LauncherConstants.forwardVelocity,
            launcherIO.getVelocity(),
            ShooterConstants.LauncherConstants.launcherAtSetpointTolerance);
      } else {
        // default shooting
        return turretAtSetpoint && hoodAtSetpoint && launcherAtSetpoint;
      }
    } else {
      return false;
    }
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
    turretIO.setPosition(ShooterConstants.TurretConstants.forwardPosition);
    hoodIO.setPosition(ShooterConstants.HoodConstants.forwardPosition);
    launcherIO.setVelocity(ShooterConstants.LauncherConstants.forwardVelocity);
  }

  private void shoot() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPose());
    launcherIO.setVelocity(shootingParameters.flywheelSpeed());
  }

  private void testShoot() {
    launcherIO.setVelocity(dashboardLauncherVelocity);
    hoodIO.setPosition(shootingParameters.hoodPose());
    turretIO.setAngle(shootingParameters.turretAngle());
  }

  private void trackTarget() {
    turretIO.setAngle(shootingParameters.turretAngle());
    launcherIO.setPower(0);
    hoodIO.setPosition(shootingParameters.hoodPose());
  }

  public boolean validShootingLocation() {
    return shootingParameters.isValid();
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

  // public void setHoodPositionDashboard() {
  //   hoodIO.setPosition(wantedHoodPosition);
  // }

  public void setLauncherPower(double power) {
    launcherIO.setPower(power);
  }

  public void setLauncherVelocity(double velocity) {
    launcherIO.setVelocity(velocity);
  }

  public void setDashboardSetpoints() {}

  public Command launcherSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> launcherIO.setLauncherOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(launcherSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command launcherSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> launcherIO.setLauncherOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(launcherSysId.dynamic(direction));
  }

  public Command turretSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> turretIO.setTurretOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(turretSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command turretSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> turretIO.setTurretOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(turretSysId.dynamic(direction));
  }
}
