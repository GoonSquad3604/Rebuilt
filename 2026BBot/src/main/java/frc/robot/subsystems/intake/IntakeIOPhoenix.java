// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
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

  //declare motors
  
  private TalonFXS armPosMotor;
  private TalonFX intakeMotor

  armPosMotor = new TalonFXS(IntakeConstants.armMotorID);
  intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
  
  // intaking functions
  void setVoltage(double voltage) {}

  void setPower(double power) {}

  // arm intake motor functions
  void setArmIntakeMotorVoltage(double voltage) {}
  void setArmIntakeMotorPower(double power) {}

  // arm functions
  void setArmPos(double pos) {}
}
