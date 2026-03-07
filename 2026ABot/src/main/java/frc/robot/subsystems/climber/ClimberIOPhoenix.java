package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class ClimberIOPhoenix implements ClimberIO {

  private final TalonFX outerMotor, innerMotor;
  private final CANcoder outerEncoder, innerEncoder;
  private final TalonFXConfiguration outerMotorConfig, innerMotorConfig;
  private final CANcoderConfiguration outerEncoderConfig, innerEncoderConfig;

  private final PositionVoltage innerRequest;
  private final PositionVoltage outerRequest;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> outerPosition;
  private final StatusSignal<AngularVelocity> outerVelocity;
  private final StatusSignal<Voltage> outerAppliedVoltage;
  private final StatusSignal<Current> outerSupplyCurrent;
  private final StatusSignal<Current> outerTorqueCurrent;
  private final StatusSignal<Temperature> outerTempCelsius;

  private final StatusSignal<Angle> innerPosition;
  private final StatusSignal<AngularVelocity> innerVelocity;
  private final StatusSignal<Voltage> innerAppliedVoltage;
  private final StatusSignal<Current> innerSupplyCurrent;
  private final StatusSignal<Current> innerTorqueCurrent;
  private final StatusSignal<Temperature> innerTempCelsius;

  public ClimberIOPhoenix() {

    innerRequest = new PositionVoltage(0).withSlot(0);
    outerRequest = new PositionVoltage(0).withSlot(0);

    // outer motor config
    outerMotor = new TalonFX(ClimberConstants.outerMotorID, Constants.CANBusName);
    outerMotorConfig = new TalonFXConfiguration();
    outerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    outerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    outerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ClimberConstants.outerP)
            .withKI(ClimberConstants.outerI)
            .withKD(ClimberConstants.outerD)
            .withKS(ClimberConstants.outerS)
            .withKV(ClimberConstants.outerV)
            .withKA(ClimberConstants.outerA);
    outerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // inner motor config
    innerMotor = new TalonFX(ClimberConstants.innerMotorID, Constants.CANBusName);
    innerMotorConfig = new TalonFXConfiguration();
    innerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    innerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    innerMotorConfig.Feedback.FeedbackRemoteSensorID = ClimberConstants.innerEncoderID;
    innerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    innerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    innerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ClimberConstants.innerP)
            .withKI(ClimberConstants.innerI)
            .withKD(ClimberConstants.innerD)
            .withKS(ClimberConstants.innerS)
            .withKV(ClimberConstants.innerV)
            .withKA(ClimberConstants.innerA);
    innerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // outer encoder config
    outerEncoder = new CANcoder(ClimberConstants.outerEncoderID, Constants.CANBusName);
    outerEncoderConfig = new CANcoderConfiguration();

    // inner encoder config
    innerEncoder = new CANcoder(ClimberConstants.innerEncoderID, Constants.CANBusName);
    innerEncoderConfig = new CANcoderConfiguration();

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> outerMotor.getConfigurator().apply(outerMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> innerMotor.getConfigurator().apply(innerMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> outerEncoder.getConfigurator().apply(outerEncoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> innerEncoder.getConfigurator().apply(innerEncoderConfig));

    // outer base status signal
    outerPosition = innerMotor.getPosition();
    outerVelocity = innerMotor.getVelocity();
    outerAppliedVoltage = innerMotor.getMotorVoltage();
    outerSupplyCurrent = innerMotor.getSupplyCurrent();
    outerTorqueCurrent = innerMotor.getTorqueCurrent();
    outerTempCelsius = innerMotor.getDeviceTemp();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                outerPosition,
                outerVelocity,
                outerAppliedVoltage,
                outerSupplyCurrent,
                outerTorqueCurrent,
                outerTempCelsius));

    // inner base status signal
    innerPosition = innerMotor.getPosition();
    innerVelocity = innerMotor.getVelocity();
    innerAppliedVoltage = innerMotor.getMotorVoltage();
    innerSupplyCurrent = innerMotor.getSupplyCurrent();
    innerTorqueCurrent = innerMotor.getTorqueCurrent();
    innerTempCelsius = innerMotor.getDeviceTemp();
    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                innerPosition,
                innerVelocity,
                innerAppliedVoltage,
                innerSupplyCurrent,
                innerTorqueCurrent,
                innerTempCelsius));

    // optimize bus utilization
    PhoenixUtil.tryUntilOk(5, () -> outerMotor.optimizeBusUtilization(0, 1.0));
    PhoenixUtil.tryUntilOk(5, () -> innerMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.outerMotorConnected = outerMotor.isConnected();
    inputs.outerEncoderConnected = outerEncoder.isConnected();
    inputs.outerVelocity = outerMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerCurrent = outerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.outerVelocity = outerMotor.getVelocity().getValueAsDouble();
    inputs.outerPosition = outerMotor.getPosition().getValueAsDouble();

    inputs.innerMotorConnected = innerMotor.isConnected();
    inputs.innerEncoderConnected = innerEncoder.isConnected();
    inputs.innerVelocity = innerMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerCurrent = innerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.innerVelocity = innerMotor.getVelocity().getValueAsDouble();
    inputs.innerPosition = innerMotor.getPosition().getValueAsDouble();
  }

  // Inner
  @Override
  public void setInnerPosition(double position) {
    innerMotor.setControl(innerRequest.withPosition(position));
  }

  @Override
  public double getInnerPosition() {
    return innerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setInnerPower(double power) {
    innerMotor.set(power);
  }

  @Override
  public void setInnerOpenLoop(double output) {
    innerMotor.setControl(voltageRequest.withOutput(output));
  }

  // Outer
  @Override
  public void setOuterPosition(double position) {
    innerMotor.setControl(outerRequest.withPosition(position));
  }

  @Override
  public double getOuterPosition() {
    return innerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setOuterPower(double power) {
    innerMotor.set(power);
  }

  @Override
  public void setOuterOpenLoop(double output) {
    innerMotor.setControl(voltageRequest.withOutput(output));
  }
}
