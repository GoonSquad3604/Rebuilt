// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
  default void updateInputs(ClimberIOInputs inputs) {}

  @AutoLog
  class ClimberIOInputs {
    public boolean outerMotorConnected = false;
    public double outerMotorVoltage;
    public double outerMotorCurrent;
    public boolean innerMotorConnected = false;
    public double innerMotorVoltage;
    public double innerMotorCurrent;

    public boolean outerEncoderConnected = false;
    public double outerPosition;
    public boolean innerEncoderConnected = false;
    public double innerPostion;
  }

  default void setPowerOuter(double power) {}

  default void setPowerInner(double power) {}

  default void setClimber1OpenLoop(double output) {}

  default void setClimber2OpenLoop(double output) {}

  default void setPositionOuter(double position) {}

  default void setPositionInner(double position) {}

  default double getPositionOuter() {
    return 0.0;
  }

  default double getPositionInner() {
    return 0.0;
  }

  default void setVoltageOuter(double volts) {}

  default void setVoltageInner(double volts) {}
}
