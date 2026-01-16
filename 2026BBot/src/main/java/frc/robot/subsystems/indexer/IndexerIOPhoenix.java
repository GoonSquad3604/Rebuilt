// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

/** Add your docs here. */
public class IndexerIOPhoenix implements IndexerIO {
  public IndexerIOPhoenix() {
    boolean indexMotorIsConnected = false;
    boolean indexToShootMotorIsConnected = false;

    double indexMotorVoltage;
    double indexToShootMotorVoltage;

    double indexMotorCurrent;
    double indexToShootMotorCurrent;

    double indexMotorTemperature;
    double indexToShootMotorTemperature;
  }

  @Override
  public void setIndexPower(double power) {}

  @Override
  public void setIndexMotorVoltage(double volts) {}

  @Override
  public void setIndexToShootMotorVoltage(double volts) {}
}
