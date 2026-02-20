// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Add your docs here. */
public class IndexerIORev implements IndexerIO {

  private final SparkMax indexMotor;
  private SparkClosedLoopController PIDController;

  private SparkFlexConfig config;

  public IndexerIORev() {
    indexMotor = new SparkMax(IndexerConstants.indexID, MotorType.kBrushless);

    PIDController = indexMotor.getClosedLoopController();
    config = new SparkFlexConfig();
    config
        .closedLoop
        .p(IndexerConstants.indexP)
        .i(IndexerConstants.indexI)
        .d(IndexerConstants.indexD)
        .outputRange(-.7, .7);

    // Set PID gains
    config
        .closedLoop
        .feedForward
        .kS(IndexerConstants.indexS)
        .kV(IndexerConstants.indexV)
        .kA(ShooterConstants.KickerConstants.kickerA);

    indexMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexVoltage = indexMotor.getBusVoltage();
    inputs.indexCurrent = indexMotor.getOutputCurrent();
    inputs.indexTemperature = indexMotor.getMotorTemperature();
    inputs.indexRPM = indexMotor.getEncoder().getVelocity();
    inputs.indexPosition = indexMotor.getEncoder().getPosition();
  }

  @Override
  public void setIndexPower(double power) {
    indexMotor.set(power);
  }

  @Override
  public void setIndexRPM(double RPM) {
    PIDController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setIndexMotorVoltage(double volts) {
    indexMotor.setVoltage(volts);
  }
}
