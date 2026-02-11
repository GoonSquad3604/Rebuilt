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
    public boolean lowRungMotorConnected = false;
    public double lowRungMotorVoltage;
    public double lowRungMotorCurrent;
    public boolean midRungMotorConnected = false;
    public double midRungMotorVoltage;
    public double midRungMotorCurrent;

    public boolean lowRungEncoderConnected = false;
    public double lowRungPosition;
    public boolean midRungEncoderConnected = false;
    public double midRungPostion;
  }

  default void setPowerLowRung(double power) {}

  default void setPowerMidRung(double power) {}

  default void setPositionLowRung(double position) {}

  default void setPositionMidRung(double position) {}

  default double getPositionLowRung() {
    return 0.0;
  }

  default double getPositionMidRung() {
    return 0.0;
  }

  default void setVoltageLowRung(double volts) {}

  default void setVoltageMidRung(double volts) {}
}
