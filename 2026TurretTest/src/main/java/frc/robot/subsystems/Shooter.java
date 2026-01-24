// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor;
  private double wantedRPM;

  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    shooterMotor = new TalonFX(8);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.shooterP;
    slot0Configs.kI = ShooterConstants.shooterI;
    slot0Configs.kD = ShooterConstants.shooterD;
    slot0Configs.kS = ShooterConstants.shooterS;
    slot0Configs.kV = ShooterConstants.shooterV;

    shooterMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter rpm", getRPM());
    wantedRPM = SmartDashboard.getNumber("wantedRPM", 0);
    SmartDashboard.putNumber("wantedRPM", wantedRPM);
  }

  public void setPower(double power) {
    shooterMotor.set(power);
  }

  public void setRPM(double RPM) {
    shooterMotor.setControl(
        m_request.withVelocity(RPM / 60)); // .withFeedForward(ShooterConstants.shooterFF));
  }

  public double getRPM() {
    return shooterMotor.getVelocity().getValueAsDouble() * 60;
  }

  public double getWantedRPM() {
    return wantedRPM;
  }
}
