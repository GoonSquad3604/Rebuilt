package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final LauncherIOInputsAutoLogged launcherInputs = new LauncherIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  public enum WantedState {
    IDLE,

    /* MANUAL SHOOTING */
    TARGET_FORWARD,
    SHOOT_FORWARD,

    /* HUB TRACKING */
    TARGET_HUB,
    SHOOT_HUB,

    /* ALLIANCE ZONE PASSING */
    TARGET_ZONE,
    SHOOT_ZONE,

    /* CORRAL PASSING */
    TARGET_CORRAL,
    SHOOT_CORRAL,
  }

  private enum CurrentState {
    IDLING,

    /* MANUAL SHOOTING */
    AIMING_FORWARD,
    REVVING_FORWARD,
    READY_TO_SHOOT_FORWARD,

    /* HUB TRACKING */
    AIMING_HUB,
    REVVING_HUB,
    READY_TO_SHOOT_HUB,

    /* ALLIANCE ZONE PASSING */
    AIMING_ZONE,
    REVVING_ZONE,
    READY_TO_SHOOT_ZONE,

    /* CORRAL PASSING */
    AIMING_CORRAL,
    REVVING_CORRAL,
    READY_TO_SHOOT_CORRAL
  }

  private WantedState wantedState = WantedState.IDLE;
  private WantedState previousWantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    synchronized (hoodInputs) {
      synchronized (launcherInputs) {
        synchronized (turretInputs) {
          Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
          Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
          Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

          currentState = handleStateTransitions();

          Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
          Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);
          Logger.recordOutput("Subsystems/Shooter/ReachedSetpoint", reachedSetpoint());

          Rotation2d wantedHoodAngle;
          double wantedLauncherRPM;
          Rotation2d wantedTurretAngle;

          applyStates();

          previousWantedState = this.wantedState;
        }
      }
    }
  }

  public CurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return CurrentState.IDLING;
      case SHOOT_CORRAL:
        return reachedSetpoint() ? CurrentState.READY_TO_SHOOT_CORRAL : CurrentState.REVVING_CORRAL;
      case SHOOT_FORWARD:
        return reachedSetpoint()
            ? CurrentState.READY_TO_SHOOT_FORWARD
            : CurrentState.REVVING_FORWARD;
      case SHOOT_HUB:
        return reachedSetpoint() ? CurrentState.READY_TO_SHOOT_HUB : CurrentState.REVVING_HUB;
      case SHOOT_ZONE:
        return reachedSetpoint() ? CurrentState.READY_TO_SHOOT_ZONE : CurrentState.REVVING_ZONE;
      case TARGET_CORRAL:
        return CurrentState.AIMING_CORRAL;
      case TARGET_FORWARD:
        return CurrentState.AIMING_FORWARD;
      case TARGET_HUB:
        return CurrentState.AIMING_HUB;
      case TARGET_ZONE:
        return CurrentState.AIMING_ZONE;
    }
    return CurrentState.IDLING;
  }

  public void applyStates() {
    switch (currentState) {
      case IDLING:
        break;
      case AIMING_CORRAL:
        break;
      case AIMING_FORWARD:
        break;
      case AIMING_HUB:
        break;
      case AIMING_ZONE:
        break;
      case READY_TO_SHOOT_CORRAL:
        break;
      case READY_TO_SHOOT_FORWARD:
        break;
      case READY_TO_SHOOT_HUB:
        break;
      case READY_TO_SHOOT_ZONE:
        break;
      case REVVING_CORRAL:
        break;
      case REVVING_FORWARD:
        break;
      case REVVING_HUB:
        break;
      case REVVING_ZONE:
        break;
      default:
        break;
    }
  }

  public boolean reachedSetpoint() {
    synchronized (hoodInputs) {
      synchronized (launcherInputs) {
        synchronized (turretInputs) {
          return false; // replace with logic for at setpoints
        }
      }
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }
}
