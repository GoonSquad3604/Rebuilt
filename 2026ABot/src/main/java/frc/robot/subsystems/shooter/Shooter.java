package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private ShootingParameters shootingParameters;
  private ShootingParameters lastParameters;

  private boolean turretAtSetpoint = false;
  private boolean hoodAtSetpoint = false;
  private boolean launcherAtSetpoint = false;

  public enum ShooterWantedState {
    IDLE,
    SHOOT,
  }

  private enum CurrentState {
    IDLING,

    /* MANUAL SHOOTING */
    REVVING_FORWARD,
    SHOOTING_FORWARD,

    REVVING,
    SHOOTING
  }

  private ShooterWantedState wantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
    this.turretIO = turretIO;
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

    Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

    Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);

    Logger.recordOutput("Subsystems/Shooter/TurretAtSetpoint", turretAtSetpoint);
    Logger.recordOutput("Subsystems/Shooter/HoodAtSetpoint", hoodAtSetpoint);
    Logger.recordOutput("Subsystems/Shooter/LauncherAtSetpoint", launcherAtSetpoint);
    Logger.recordOutput("Subsystems/Shooter/ReachedSetpoint", reachedSetpoints());

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
      Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
      // applyStates();
    }
    RobotState.getInstance().setTurretAngle(Rotation2d.fromDegrees(turretIO.getAngle()));
  }

  public CurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return CurrentState.IDLING;
      case SHOOT:
        if (!(RobotState.getInstance().isOverride())) {
          return reachedSetpoints() ? CurrentState.SHOOTING : CurrentState.REVVING;
        } else if (RobotState.getInstance().getTarget() != ShooterTarget.FORWARD) {
          return reachedSetpoints() ? CurrentState.SHOOTING : CurrentState.REVVING;
        } else {
          return reachedSetpoints() ? CurrentState.SHOOTING_FORWARD : CurrentState.REVVING_FORWARD;
        }
    }
    return CurrentState.IDLING;
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
      default:
        break;
    }
  }

  public boolean reachedSetpoints() {
    if (shootingParameters != null && wantedState == ShooterWantedState.SHOOT) {

      // check systems for if at desired positions
      turretAtSetpoint = MathUtil.isNear(shootingParameters.turretAngle(), turretIO.getAngle(), 20);
      hoodAtSetpoint = MathUtil.isNear(shootingParameters.hoodPose(), hoodIO.getPosition(), 0.2);
      launcherAtSetpoint =
          MathUtil.isNear(shootingParameters.flywheelSpeed(), launcherIO.getVelocity(), 20);

      if (RobotState.getInstance().isOverride()
          && RobotState.getInstance().getTarget() == ShooterTarget.FORWARD) {
        // if overriding for forward shot
        return MathUtil.isNear(45, launcherIO.getVelocity(), 2);
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
    turretIO.setAngle(ShooterConstants.TurretConstants.forwardPosition);
    hoodIO.setPosition(ShooterConstants.HoodConstants.forwardPosition);
    launcherIO.setVelocity(ShooterConstants.LauncherConstants.forwardVelocity);
  }

  private void shoot() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPose());
    launcherIO.setVelocity(shootingParameters.flywheelSpeed());
  }

  // testcontroller:
  public void setTurretPower(double power) {
    turretIO.setPower(power);
  }

  public void setTurretPos(double position) {
    turretIO.setAngle(position);
  }

  public void setHoodPower(double power) {
    hoodIO.setPower(power);
  }

  public void setLauncherPower(double power) {
    launcherIO.setPower(power);
  }

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
