// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  private SparkMax turretMotor;
  private RelativeEncoder turretEncoder;

  public Turret() {

    turretMotor = new SparkMax(7, MotorType.kBrushless);
    turretEncoder = turretMotor.getEncoder();
    
  }

  public void turnCounterClockwise() {
    turretMotor.set(.3);
  }

  public void turnClockwise() {
    turretMotor.set(-.3);
  }

  public void stopTurret() {
    turretMotor.set(0);
  }

  public void setPosition(double position) {
    turretEncoder.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
