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

  // logging
  public boolean intakeMotorConnected;

  TalonFXS intakeMotor = new TalonFXS(IntakeConstants.intakeMotorID);

  // intaking functions
  void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  void setPower(double power) {
    intakeMotor.set(power);
  }

}
