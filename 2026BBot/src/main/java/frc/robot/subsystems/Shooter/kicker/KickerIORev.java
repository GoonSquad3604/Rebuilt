// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.subsystems.shooter.ShooterConstants;

/** Add your docs here. */
public class KickerIORev implements KickerIO {

  private final SparkMax kickerMotor;
  private SparkClosedLoopController PIDController;

  private SparkFlexConfig config;

  public KickerIORev() {
    kickerMotor = new SparkMax(ShooterConstants.KickerConstants.kickerID, MotorType.kBrushless);

    PIDController = kickerMotor.getClosedLoopController();
    config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config
        .closedLoop
        .p(ShooterConstants.KickerConstants.kickerP)
        .i(ShooterConstants.KickerConstants.kickerI)
        .d(ShooterConstants.KickerConstants.kickerD)
        .outputRange(-.7, .7);

    // Set PID gains
    config
        .closedLoop
        .feedForward
        .kS(ShooterConstants.KickerConstants.kickerS)
        .kV(ShooterConstants.KickerConstants.kickerV)
        .kA(ShooterConstants.KickerConstants.kickerA);

    kickerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void updateInputs(KickerIOInputs inputs) {
    inputs.voltage = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
    inputs.velocity = kickerMotor.getEncoder().getVelocity();
    inputs.current = kickerMotor.getOutputCurrent();
    inputs.temperature = kickerMotor.getMotorTemperature();
    inputs.position = kickerMotor.getEncoder().getPosition();
  }

  @Override
  public void setPower(double power) {
    kickerMotor.set(power);
  }

  @Override
  public void setVelocity(double velocity) {
    PIDController.setSetpoint(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setKickerOpenLoop(double output) {
    kickerMotor.setVoltage(output);
  }

  @Override
  public void setVoltage(double volts) {
    kickerMotor.setVoltage(volts);
  }
}
