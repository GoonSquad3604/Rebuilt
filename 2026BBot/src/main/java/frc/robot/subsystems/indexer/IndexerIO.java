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
    public boolean indexToShootMotorIsConnected = false;

    public double indexVoltage;
    public double indexToShootVoltage;

    public double indexPower;
    public double indexToShootPower;

    public double indexCurrent;
    public double indexToShootCurrent;

    public double indexTemperature;
    public double indexToShootTemperature;
  }

  public void setIndexPower(double power);

  public void setIndexMotorVoltage(double volts);

  public void setIndexToShootMotorVoltage(double volts);
}
