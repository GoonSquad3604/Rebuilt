// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ClimberIOPhoenix implements ClimberIO {
    private TalonFX lowRungMotor, midRungMotor;
    private CANcoder lowRungEncoder, midRungEncoder;

    private TalonFXConfiguration lowRungConfig, midRungConfig;
    private CANcoderConfiguration lowRungEncoderConfig, midRungEncoderConfig;

    public ClimberIOPhoenix() {
        //declared motor & configs
        lowRungMotor = new TalonFX(ClimberConstants.lowRungMotorID);
        midRungMotor = new TalonFX(ClimberConstants.midRungMotorID);
        lowRungEncoder = new CANcoder(ClimberConstants.lowRungEncoderID);
        midRungEncoder = new CANcoder(ClimberConstants.midRungEncoderID);

        lowRungConfig = new TalonFXConfiguration();
        midRungConfig = new TalonFXConfiguration();
        lowRungEncoderConfig = new CANcoderConfiguration();
        midRungEncoderConfig = new CANcoderConfiguration();

        //configs for Encoders
        lowRungEncoder.getConfigurator().apply(lowRungEncoderConfig);
        midRungEncoder.getConfigurator().apply(midRungEncoderConfig);

        //configs for both motors
        lowRungConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //placeholder
        lowRungConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //placeholder
        lowRungConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //placeholder
        lowRungConfig.CurrentLimits.SupplyCurrentLimit = 40; //placeholder
        lowRungConfig.CurrentLimits.StatorCurrentLimitEnable = true; //placeholder
        lowRungConfig.CurrentLimits.StatorCurrentLimit = 80; //placeholder
        lowRungConfig.Voltage.PeakForwardVoltage = 12.0; //placeholder
        lowRungConfig.Voltage.PeakReverseVoltage = -12.0; //placeholder
        lowRungConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02; //placeholder
        lowRungConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5; //placeholder
        
        midRungConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //placeholder
        midRungConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //placeholder
        midRungConfig.CurrentLimits.SupplyCurrentLimitEnable = true; //placeholder
        midRungConfig.CurrentLimits.SupplyCurrentLimit = 40; //placeholder
        midRungConfig.CurrentLimits.StatorCurrentLimitEnable = true; //placeholder
        midRungConfig.CurrentLimits.StatorCurrentLimit = 80; //placeholder
        midRungConfig.Voltage.PeakForwardVoltage = 12.0; //placeholder
        midRungConfig.Voltage.PeakReverseVoltage = -12.0; //placeholder
        midRungConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02; //placeholder
        midRungConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .5; //placeholder

        //apply configs
        PhoenixUtil.tryUntilOk(5, () -> lowRungMotor.getConfigurator().apply(lowRungConfig));
        PhoenixUtil.tryUntilOk(5, () -> midRungMotor.getConfigurator().apply(midRungConfig));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.lowRungMotorConnected =
        BaseStatusSignal.refreshAll(
                lowRungMotor.getMotorVoltage(),
                lowRungMotor.getSupplyCurrent(),
                lowRungMotor.getDeviceTemp(),
                lowRungMotor.getVelocity())
            .isOK();
    inputs.lowRungMotorVoltage = lowRungMotor.getMotorVoltage().getValueAsDouble();
    inputs.lowRungMotorCurrent = lowRungMotor.getSupplyCurrent().getValueAsDouble();
    
    inputs.midRungMotorConnected =
        BaseStatusSignal.refreshAll(
                midRungMotor.getMotorVoltage(),
                midRungMotor.getSupplyCurrent(),
                midRungMotor.getDeviceTemp(),
                midRungMotor.getVelocity())
            .isOK();
    inputs.midRungMotorVoltage = midRungMotor.getMotorVoltage().getValueAsDouble();
    inputs.midRungMotorCurrent = midRungMotor.getSupplyCurrent().getValueAsDouble();

    inputs.lowRungEncoderConnected = lowRungEncoder.isConnected();
    inputs.lowRungPosition = lowRungEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.midRungEncoderConnected = midRungEncoder.isConnected();
    inputs.midRungPostion = midRungEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setPowerLowRung(double power){
        lowRungMotor.set(power);
    }

    @Override
    public void setPowerMidRung(double power){
        midRungMotor.set(power);
    }

    @Override
    public double getPositionLowRung(){
        return lowRungEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public double getPositionMidRung(){
        return midRungEncoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setVoltageLowRung(double voltage){
        lowRungMotor.setVoltage(voltage);
    }

    @Override
    public void setVoltageMidRung(double voltage){
        midRungMotor.setVoltage(voltage);
    }
}
