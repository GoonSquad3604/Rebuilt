// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ClimberIOPhoenix implements ClimberIO {
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private TalonFX outerMotor, innerMotor;
  private CANcoder outerEncoder, innerEncoder;
  private TalonFXConfiguration outerConfig, innerConfig;
  private CANcoderConfiguration outerEncoderConfig, innerEncoderConfig;

  public ClimberIOPhoenix() {
    // declared motor & configs
    outerMotor = new TalonFX(ClimberConstants.climberHook1MotorID, Constants.CANBusName);
    innerMotor = new TalonFX(ClimberConstants.climberHook2MotorID, Constants.CANBusName);
    outerEncoder = new CANcoder(ClimberConstants.outerEncoderID, Constants.CANBusName);
    innerEncoder = new CANcoder(ClimberConstants.innerEncoderID, Constants.CANBusName);

    outerConfig = new TalonFXConfiguration();
    innerConfig = new TalonFXConfiguration();
    outerEncoderConfig = new CANcoderConfiguration();
    innerEncoderConfig = new CANcoderConfiguration();

    // configs for Encoders
    outerEncoder.getConfigurator().apply(outerEncoderConfig);
    innerEncoder.getConfigurator().apply(innerEncoderConfig);

    // configs for both motors
    outerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // placeholder
    outerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // placeholder
    outerConfig.CurrentLimits.SupplyCurrentLimitEnable = true; // placeholder
    outerConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder
    outerConfig.CurrentLimits.StatorCurrentLimitEnable = true; // placeholder
    outerConfig.CurrentLimits.StatorCurrentLimit = 80; // placeholder
    outerConfig.Voltage.PeakForwardVoltage = 12.0; // placeholder
    outerConfig.Voltage.PeakReverseVoltage = -12.0; // placeholder
    outerConfig.Slot0 =
        new Slot0Configs()
            .withKP(ClimberConstants.innerP)
            .withKI(ClimberConstants.innerI)
            .withKD(ClimberConstants.innerD)
            .withKS(ClimberConstants.innerS)
            .withKV(ClimberConstants.innerV)
            .withKA(ClimberConstants.innerA);
    outerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02; // placeholder
    outerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5; // placeholder

    innerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; // placeholder
    innerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // placeholder
    innerConfig.CurrentLimits.SupplyCurrentLimitEnable = true; // placeholder
    innerConfig.CurrentLimits.SupplyCurrentLimit = 40; // placeholder
    innerConfig.CurrentLimits.StatorCurrentLimitEnable = true; // placeholder
    innerConfig.CurrentLimits.StatorCurrentLimit = 80; // placeholder
    innerConfig.Voltage.PeakForwardVoltage = 12.0; // placeholder
    innerConfig.Voltage.PeakReverseVoltage = -12.0; // placeholde
    innerConfig.Slot0 =
        new Slot0Configs()
            .withKP(ClimberConstants.innerP)
            .withKI(ClimberConstants.innerI)
            .withKD(ClimberConstants.innerD)
            .withKS(ClimberConstants.innerS)
            .withKV(ClimberConstants.innerV)
            .withKA(ClimberConstants.innerA);
    innerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02; // placeholder
    innerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5; // placeholder

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> outerMotor.getConfigurator().apply(outerConfig));
    PhoenixUtil.tryUntilOk(5, () -> innerMotor.getConfigurator().apply(innerConfig));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.outerMotorConnected =
        BaseStatusSignal.refreshAll(
                outerMotor.getMotorVoltage(),
                outerMotor.getSupplyCurrent(),
                outerMotor.getDeviceTemp(),
                outerMotor.getVelocity())
            .isOK();
    inputs.outerMotorVoltage = outerMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerMotorCurrent = outerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.innerMotorConnected =
        BaseStatusSignal.refreshAll(
                innerMotor.getMotorVoltage(),
                innerMotor.getSupplyCurrent(),
                innerMotor.getDeviceTemp(),
                innerMotor.getVelocity())
            .isOK();
    inputs.innerMotorVoltage = innerMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerMotorCurrent = innerMotor.getSupplyCurrent().getValueAsDouble();

    inputs.outerEncoderConnected = outerEncoder.isConnected();
    inputs.outerPosition = outerEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.innerEncoderConnected = innerEncoder.isConnected();
    inputs.innerPostion = innerEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setPowerOuter(double power) {
    outerMotor.set(power);
  }

  @Override
  public void setPowerInner(double power) {
    innerMotor.set(power);
  }

  @Override
  public double getPositionOuter() {
    return outerEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public double getPositionInner() {
    return innerEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setVoltageOuter(double voltage) {
    outerMotor.setVoltage(voltage);
  }

  @Override
  public void setVoltageInner(double voltage) {
    innerMotor.setVoltage(voltage);
  }

  @Override
  public void setClimber1OpenLoop(double output) {
    outerMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setClimber2OpenLoop(double output) {
    innerMotor.setControl(voltageRequest.withOutput(output));
  }
}
