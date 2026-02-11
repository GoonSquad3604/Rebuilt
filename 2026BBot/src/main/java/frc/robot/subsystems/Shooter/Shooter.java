package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.shooter.launcher.LauncherIO;
import frc.robot.subsystems.shooter.launcher.LauncherIOInputsAutoLogged;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final HoodIO hoodIO;
  private final LauncherIO launcherIO;
  private final TurretIO turretIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final LauncherIOInputsAutoLogged launcherInputs = new LauncherIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  @AutoLogOutput private double wantedHoodAngle;
  @AutoLogOutput private double wantedLauncherRPM;
  @AutoLogOutput private double wantedTurretAngle;

  public enum ShooterWantedState {
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
    REVVING_FORWARD,
    SHOOTING_FORWARD,

    /* HUB TRACKING */
    REVVING_HUB,
    SHOOTING_HUB,

    /* ALLIANCE ZONE PASSING */
    REVVING_ZONE,
    SHOOTING_ZONE,

    /* CORRAL PASSING */
    REVVING_CORRAL,
    SHOOTING_CORRAL
  }

  private ShooterWantedState wantedState = ShooterWantedState.IDLE;
  private ShooterWantedState previousWantedState = ShooterWantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLING;

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    // synchronized (hoodInputs) {
    //   synchronized (launcherInputs) {
    //     synchronized (turretInputs) {
    Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);

    currentState = handleStateTransitions();

    Logger.recordOutput("Subsystems/Shooter/CurrentState", currentState);
    Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);
    Logger.recordOutput("Subsystems/Shooter/ReachedSetpoint", reachedSetpoint());

    applyStates();

    previousWantedState = this.wantedState;
    // }
    //     }
    //   }
  }

  public CurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return CurrentState.IDLING;
      case SHOOT_CORRAL:
        return reachedSetpoint() ? CurrentState.SHOOTING_CORRAL : CurrentState.REVVING_CORRAL;
      case SHOOT_FORWARD:
        return reachedSetpoint() ? CurrentState.SHOOTING_FORWARD : CurrentState.REVVING_FORWARD;
      case SHOOT_HUB:
        return reachedSetpoint() ? CurrentState.SHOOTING_HUB : CurrentState.REVVING_HUB;
      case SHOOT_ZONE:
        return reachedSetpoint() ? CurrentState.SHOOTING_ZONE : CurrentState.REVVING_ZONE;
    }
    return CurrentState.IDLING;
  }

  public void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case SHOOTING_CORRAL:
        shootCorral();
        break;
      case SHOOTING_FORWARD:
        shootForward();
        break;
      case SHOOTING_HUB:
        shootHub();
        break;
      case SHOOTING_ZONE:
        shootZone();
        break;
      case REVVING_CORRAL:
        revCorral();
        break;
      case REVVING_FORWARD:
        revForward();
        break;
      case REVVING_HUB:
        revHub();
        break;
      case REVVING_ZONE:
        revZone();
        break;
      default:
        break;
    }

    hoodIO.setAngle(wantedHoodAngle);
    launcherIO.setRPM(wantedLauncherRPM);
    turretIO.setAngle(wantedTurretAngle);
  }

  public boolean reachedSetpoint() {
    // synchronized (hoodInputs) {
    //   synchronized (launcherInputs) {
    //     synchronized (turretInputs) {
    return false; // replace with logic for at setpoints
    //     }
    //   }
    // }
  }

  public void setWantedState(ShooterWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void idling() {
    // hoodIO.setPower(0);
    // launcherIO.setPower(0);
    // turretIO.setPower(0);
  }

  private void shootCorral() {}

  private void shootForward() {}

  private void shootHub() {}

  private void shootZone() {}

  private void revCorral() {}

  private void revForward() {}

  private void revHub() {}

  private void revZone() {}

  // testcontroller:
  public void setLauncherPower(double power) {
    launcherIO.setPower(power);
  }

  public void setHoodPower(double power) {
    hoodIO.setPower(power);
  }

  public void setTurretPower(double power) {
    turretIO.setPower(power);
  }
}
