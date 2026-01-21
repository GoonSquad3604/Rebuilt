// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor;

  public Shooter() {
    shooterMotor = new TalonFX(8);
  }

  @Override
  public void periodic() {}

  public void setPower(double power) {
    shooterMotor.set(power);
  }
}
