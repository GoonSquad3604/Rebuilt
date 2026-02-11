// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** Add your docs here. */
public class IndexerIORev implements IndexerIO {

  private final Spark indexMotor;

  public IndexerIORev() {
    indexMotor = new Spark(IndexerConstants.indexID);

    boolean indexMotorIsConnected = false;

    double indexVoltage;

    double indexPower;

    double indexCurrent;

    double indexTemperature;
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexVoltage = indexMotor.getVoltage();
  }

  @Override
  public void setIndexPower(double power) {}

  @Override
  public void setIndexMotorVoltage(double volts) {}
}
