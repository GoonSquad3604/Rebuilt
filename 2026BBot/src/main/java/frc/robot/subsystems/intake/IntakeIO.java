// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

public interface IntakeIO {

  class IntakeIOInputs {

    // intaking motor variables
    public double intakeMotorSpeed;
    public double intakeMotorVoltage;
    public double intakeMotorCurrent;

    // logging
    public boolean intakeMotorConnected;

    // intaking functions
    void setIntakeVoltage(double voltage) {}

    void setIntakePower(double power) {}

  }
}
