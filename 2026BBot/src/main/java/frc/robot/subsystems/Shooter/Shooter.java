package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.RobotState.ShotTarget;
import frc.robot.subsystems.shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.kicker.KickerIO;
import frc.robot.subsystems.shooter.kicker.KickerIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final HoodIO hoodIO;
  private final LauncherIO launcherIO;
  private final TurretIO turretIO;
  private final KickerIO kickerIO;

  private final SysIdRoutine launcherSysId;
  private final SysIdRoutine kickerSysId;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final LauncherIOInputsAutoLogged launcherInputs = new LauncherIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  private ShootingParameters shootingParameters;
  private ShotTarget target;

  private double wantedHoodAngle;
  private double wantedLauncherVelocity;
  private double wantedTurretAngle;

  private boolean turretAtSetpoint = false;
  private boolean hoodAtSetpoint = false;
  private boolean launcherAtSetpoint = false;

  public enum ShooterWantedState {
    IDLE,
    SHOOT,
    SHOOT_FORWARD
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
  private ShooterWantedState previousWantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO, KickerIO kickerIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
    this.turretIO = turretIO;
    this.kickerIO = kickerIO;
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

    kickerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Subsystems/Shooter/Kicker/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> kickerIO.setKickerOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    synchronized (hoodInputs) {
      synchronized (launcherInputs) {
        synchronized (turretInputs) {
          synchronized (kickerInputs) {
            hoodIO.updateInputs(hoodInputs);
            launcherIO.updateInputs(launcherInputs);
            turretIO.updateInputs(turretInputs);
            kickerIO.updateInputs(kickerInputs);
            Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
            Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
            Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);
            Logger.processInputs("Subsystems/Shooter/Kicker", kickerInputs);

            currentState = handleStateTransitions();

            Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
            Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);
            Logger.recordOutput("Subsystems/Shooter/TurretAtSetpoint", turretAtSetpoint);
            Logger.recordOutput("Subsystems/Shooter/HoodAtSetpoint", hoodAtSetpoint);
            Logger.recordOutput("Subsystems/Shooter/LauncherAtSetpoint", launcherAtSetpoint);
            Logger.recordOutput("Subsystems/Shooter/ReachedSetpoint", reachedSetpoint());

            RobotState.getInstance().setTarget();
            shootingParameters = ShotCalculator.getInstance().getParameters();
            Logger.recordOutput(
                "Subsystems/Shooter/WantedTurretAngle", shootingParameters.turretAngle());
            Logger.recordOutput("Subsystems/Shooter/Target", RobotState.getInstance().getTarget());

            applyStates();

            previousWantedState = this.wantedState;
          }
        }
      }
    }
  }

  public CurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return CurrentState.IDLING;
      case SHOOT:
        return reachedSetpoint() ? CurrentState.SHOOTING : CurrentState.REVVING;
      case SHOOT_FORWARD:
        return reachedSetpoint() ? CurrentState.SHOOTING_FORWARD : CurrentState.REVVING_FORWARD;
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
      case REVVING_FORWARD:
        revForward();
        break;
      case REVVING:
        rev();
      default:
        break;
    }
  }

  public boolean reachedSetpoint() {
    if (shootingParameters != null) {
      turretAtSetpoint = MathUtil.isNear(shootingParameters.turretAngle(), turretIO.getAngle(), 1);
      launcherAtSetpoint = MathUtil.isNear(50, launcherIO.getVelocity(), 3);
      hoodAtSetpoint = MathUtil.isNear(shootingParameters.hoodPos(), hoodIO.getPosition(), 0.05);
      if (wantedState == ShooterWantedState.SHOOT) {
        return turretAtSetpoint && launcherAtSetpoint && hoodAtSetpoint;
      } else {
        return MathUtil.isNear(50, launcherIO.getVelocity(), 3)
            && MathUtil.isNear(0, turretIO.getAngle(), 1)
            && MathUtil.isNear(0, hoodIO.getPosition(), 0.05);
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
    kickerIO.setPower(0);
  }

  private void shootForward() {
    wantedTurretAngle = 0;
    turretIO.setPosition(wantedTurretAngle);
    wantedHoodAngle = 0.0;
    hoodIO.setPosition(wantedHoodAngle);
    wantedLauncherVelocity = 50;
    launcherIO.setVelocity(wantedLauncherVelocity);
    kickerIO.setVelocity(5427.2);
  }

  private void shoot() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPos());
    wantedLauncherVelocity = 50;
    launcherIO.setVelocity(wantedLauncherVelocity);
    kickerIO.setVelocity(5427.2);
  }

  private void rev() {
    turretIO.setAngle(shootingParameters.turretAngle());
    hoodIO.setPosition(shootingParameters.hoodPos());
    wantedLauncherVelocity = 50;
    launcherIO.setVelocity(wantedLauncherVelocity);
  }

  private void revForward() {
    turretIO.setAngle(0);
    hoodIO.setPosition(shootingParameters.hoodPos());
    wantedLauncherVelocity = 50;
    launcherIO.setVelocity(wantedLauncherVelocity);
  }

  // testcontroller:
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

  public Command kickerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> kickerIO.setKickerOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(kickerSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command kickerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> kickerIO.setKickerOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(kickerSysId.dynamic(direction));
  }
}
