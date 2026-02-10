// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {

  default void updateInputs(IndexerIOInputs inputs) {}

  @AutoLog
  class IndexerIOInputs {
    public boolean indexMotorIsConnected = false;

    public double indexVoltage;

    public double indexPower;

    public double indexCurrent;

    public double indexTemperature;

    public Object indexMotorVoltage;
  }

  public void setIndexPower(double power);

  public void setIndexMotorVoltage(double volts);
}
