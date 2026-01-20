// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFXS;

public class IntakeIOPhoenix implements IntakeIO {

  // intaking motor variables
  public double intakeMotorSpeed;
  public double intakeMotorVoltage;
  public double intakeMotorCurrent;

  // arm functions
  public double armPos;
  public double armMotorVoltage;
  public double armMotorCurrent;

  // logging
  public boolean intakeMotorConnected;
  public boolean armMotorConnected;

  TalonFXS intakeMotor = new TalonFXS(IntakeConstants.intakeMotorID);
  TalonFXS armMotor = new TalonFXS(IntakeConstants.armMotorID);

  // intaking functions
  void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  void setPower(double power) {
    intakeMotor.set(power);
  }

  // arm functions
  void setArmPos(double pos) {
    armMotor.setPosition(pos);
  }
}
