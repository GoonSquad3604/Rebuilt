// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Launcher.LauncherIO;
import frc.robot.subsystems.Shooter.Launcher.LauncherIOInputsAutoLogged;
import frc.robot.subsystems.Shooter.Turret.TurretIO;
import frc.robot.subsystems.Shooter.Turret.TurretIOInputsAutoLogged;
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
    REV_FORWARD,
    SHOOT_FORWARD,

    /* HUB TRACKING */
    TARGET_HUB,
    REV_HUB,
    SHOOT_HUB,

    /* ALLIANCE ZONE PASSING */
    TARGET_ZONE,
    REV_ZONE,
    SHOOT_ZONE,

    /* CORRAL PASSING */
    TARGET_CORRAL,
    REV_CORRAL,
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

  /** Creates a new Shooter. */
  public Shooter(HoodIO hoodIO, LauncherIO launcherIO, TurretIO turretIO) {
    this.hoodIO = hoodIO;
    this.launcherIO = launcherIO;
    this.turretIO = turretIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Logger.processInputs("Subsystems/Shooter/Hood", hoodInputs);
    Logger.processInputs("Subsystems/Shooter/Launcher", launcherInputs);
    Logger.processInputs("Subsystems/Shooter/Turret", turretInputs);
  }
}
