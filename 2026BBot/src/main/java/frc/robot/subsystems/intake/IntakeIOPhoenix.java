// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOPhoenix implements IntakeIO {

  // intaking motor variables
  public double intakeMotorSpeed;
  public double intakeMotorVoltage;
  public double intakeMotorCurrent;

  // logging
  public boolean intakeMotorConnected;

  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeMotorConfig;

  // status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public IntakeIOPhoenix() {
    intakeMotorConfig = new TalonFXConfiguration();
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, Constants.CANBusName);

    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.intakeP)
            .withKI(IntakeConstants.intakeI)
            .withKD(IntakeConstants.intakeD);
    intakeMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeMotorConfig));

    position = intakeMotor.getPosition();
    velocity = intakeMotor.getVelocity();
    appliedVoltage = intakeMotor.getMotorVoltage();
    supplyCurrent = intakeMotor.getSupplyCurrent();
    torqueCurrent = intakeMotor.getTorqueCurrent();
    tempCelsius = intakeMotor.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius));
    PhoenixUtil.tryUntilOk(5, () -> intakeMotor.optimizeBusUtilization(0, 1.0));

    var slot0Configs = new Slot0Configs();

    slot0Configs.kP = IntakeConstants.intakeP;
    slot0Configs.kI = IntakeConstants.intakeI;
    slot0Configs.kD = IntakeConstants.intakeD;
    slot0Configs.kS = IntakeConstants.intakeS;
    slot0Configs.kV = IntakeConstants.intakeV;

    intakeMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected = intakeMotor.isConnected();
    inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeMotorCurrent = intakeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.intakeMotorTemp = intakeMotor.getDeviceTemp().getValueAsDouble();
  }

  // intaking functions
  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void setPower(double power) {
    intakeMotor.set(power);
  }
}
