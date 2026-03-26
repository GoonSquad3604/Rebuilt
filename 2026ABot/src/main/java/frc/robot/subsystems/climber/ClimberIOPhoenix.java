package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
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
  private final CANrange leftClimbRange, rightClimbRange, centerClimbRange;
  private final TalonFXConfiguration outerMotorConfig, innerMotorConfig;
  private final CANcoderConfiguration outerEncoderConfig, innerEncoderConfig;
  private final CANrangeConfiguration leftClimbConfig, rightClimbConfig, centerClimbConfig;

  private final MotionMagicVoltage outerRequest = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage innerRequest = new MotionMagicVoltage(0.0);

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

    // outer motor config
    outerMotor = new TalonFX(ClimberConstants.outerMotorID, Constants.CANBusName);
    leftClimbRange = new CANrange(ClimberConstants.leftClimberRangeID, Constants.CANBusName);
    rightClimbRange = new CANrange(ClimberConstants.rightClimberRangeID, Constants.CANBusName);
    centerClimbRange = new CANrange(ClimberConstants.centerClimberRangeID, Constants.CANBusName);

    leftClimbConfig = new CANrangeConfiguration();
    rightClimbConfig = new CANrangeConfiguration();
    centerClimbConfig = new CANrangeConfiguration();

    leftClimbConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    leftClimbConfig.ProximityParams.ProximityThreshold = 0.6;
    leftClimbConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;

    rightClimbConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    rightClimbConfig.ProximityParams.ProximityThreshold = 0.6;
    rightClimbConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;

    centerClimbConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500;
    centerClimbConfig.ProximityParams.ProximityThreshold = 0.6;
    centerClimbConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;

    leftClimbRange.getConfigurator().apply(leftClimbConfig);
    rightClimbRange.getConfigurator().apply(rightClimbConfig);
    centerClimbRange.getConfigurator().apply(centerClimbConfig);

    outerMotorConfig = new TalonFXConfiguration();
    outerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    outerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    outerMotorConfig.Feedback.FeedbackRemoteSensorID = ClimberConstants.outerEncoderID;
    outerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    outerMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
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
    innerMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
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
    outerEncoderConfig.MagnetSensor.MagnetOffset = 0.2; // 0.8;
    outerEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    outerEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    // inner encoder config
    innerEncoder = new CANcoder(ClimberConstants.innerEncoderID, Constants.CANBusName);
    innerEncoderConfig = new CANcoderConfiguration();
    innerEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    // motion magic
    innerMotorConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.innerAcceleration;
    innerMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.innerVelocity;

    outerMotorConfig.MotionMagic.MotionMagicAcceleration = ClimberConstants.outerAcceleration;
    outerMotorConfig.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.outerVelocity;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> outerMotor.getConfigurator().apply(outerMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> innerMotor.getConfigurator().apply(innerMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> outerEncoder.getConfigurator().apply(outerEncoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> innerEncoder.getConfigurator().apply(innerEncoderConfig));

    // outer base status signal
    outerPosition = outerEncoder.getAbsolutePosition();
    outerVelocity = outerEncoder.getVelocity();
    outerAppliedVoltage = outerMotor.getMotorVoltage();
    outerSupplyCurrent = outerMotor.getSupplyCurrent();
    outerTorqueCurrent = outerMotor.getTorqueCurrent();
    outerTempCelsius = outerMotor.getDeviceTemp();
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
    innerPosition = innerEncoder.getAbsolutePosition();
    innerVelocity = innerEncoder.getVelocity();
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
    inputs.outerVoltage = outerMotor.getMotorVoltage().getValueAsDouble();
    inputs.outerCurrent = outerMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.outerVelocity = outerEncoder.getVelocity().getValueAsDouble();
    inputs.outerPosition = outerEncoder.getPosition().getValueAsDouble();

    inputs.innerMotorConnected = innerMotor.isConnected();
    inputs.innerEncoderConnected = innerEncoder.isConnected();
    inputs.innerVoltage = innerMotor.getMotorVoltage().getValueAsDouble();
    inputs.innerCurrent = innerMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.innerVelocity = innerEncoder.getVelocity().getValueAsDouble();
    inputs.innerPosition = innerEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.leftClimbRangeConnected = leftClimbRange.isConnected();
    inputs.rightClimbRangeConnected = rightClimbRange.isConnected();
    inputs.centerClimbRangeConnected = centerClimbRange.isConnected();
  }

  // Inner
  @Override
  public void setInnerPosition(double position) {
    innerMotor.setControl(innerRequest.withPosition(position).withEnableFOC(true));
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
    outerMotor.setControl(outerRequest.withPosition(position).withEnableFOC(true));
  }

  @Override
  public double getOuterPosition() {
    return outerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setOuterPower(double power) {
    outerMotor.set(power);
  }

  @Override
  public void setOuterOpenLoop(double output) {
    outerMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public boolean leftClimbDetected() {
    return leftClimbRange.getIsDetected().getValue();
  }

  @Override
  public boolean rightClimbDetected() {
    return rightClimbRange.getIsDetected().getValue();
  }

  @Override
  public boolean centerClimbDetected() {
    return centerClimbRange.getIsDetected().getValue();
  }
}
