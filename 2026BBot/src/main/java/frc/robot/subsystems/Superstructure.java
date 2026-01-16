// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class Superstructure extends SubsystemBase {

  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Climber climber;
  private final LED led;

  public enum WantedSuperState {
    IDLE,
    INTAKE_AND_AIM_FORWARD,
    INTAKE_AND_AIM_HUB,
    INTAKE_AND_AIM_ZONE,
    INTAKE_AND_AIM_CORRAL,
    VOMIT,
    INTAKE_AND_SHOOT_FORWARD,
    INTAKE_AND_SHOOT_HUB,
    INTAKE_AND_SHOOT_ZONE,
    INTAKE_AND_SHOOT_CORRAL,
    AIM_FORWARD,
    AIM_HUB,
    AIM_ZONE,
    AIM_CORRAL,
    SHOOT_FORWARD,
    SHOOT_HUB,
    SHOOT_ZONE,
    SHOOT_CORRAL,
    CLIMB, // TBD
  }

  public enum CurrentSuperState {
    IDLING,
    INTAKING_AND_AIMING_FORWARD,
    INTAKING_AND_AIMING_HUB,
    INTAKING_AND_AIMING_ZONE,
    INTAKING_AND_AIMING_CORRAL,
    VOMITING,
    INTAKING_AND_SHOOTING_FORWARD,
    INTAKING_AND_SHOOTING_HUB,
    INTAKING_AND_SHOOTING_ZONE,
    INTAKING_AND_SHOOTING_CORRAL,
    AIMING_FORWARD,
    AIMING_HUB,
    AIMING_ZONE,
    AIMING_CORRAL,
    SHOOTING_FORWARD,
    SHOOTING_HUB,
    SHOOTING_ZONE,
    SHOOTING_CORRAL,
    CLIMBING, // TBD
  }

  /** Creates a new Superstructure. */
  public Superstructure(
      Drive drive, Intake intake, Indexer indexer, Shooter shooter, Climber climber, LED led) {
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.climber = climber;
    this.led = led;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
