package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

public class HoodIOPhoenix implements HoodIO {

  // motor
  private final TalonFX hoodMotor;
  private final PositionVoltage hoodRequest;
  private final TalonFXConfiguration hoodMotorConfig;

  // encoder
  private final CANcoder hoodEncoder;
  private final CANcoderConfiguration hoodEncoderConfig;

  // status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  
  public HoodIOPhoenix() {

    hoodMotor = new TalonFX(ShooterConstants.HoodConstants.hoodID);
    hoodRequest = new PositionVoltage(0);

    hoodEncoder  = new CANcoder(ShooterConstants.HoodConstants.hoodEncoderID);
    hoodEncoderConfig = new CANcoderConfiguration();

    hoodEncoder.getConfigurator().apply(hoodEncoderConfig);

    hoodMotorConfig = new TalonFXConfiguration();

    hoodMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    hoodMotorConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.HoodConstants.hoodEncoderID;
    hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    hoodMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.HoodConstants.hoodP)
            .withKI(ShooterConstants.HoodConstants.hoodI)
            .withKD(ShooterConstants.HoodConstants.hoodD);
    hoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodMotorConfig));

    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    appliedVoltage = hoodMotor.getMotorVoltage();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    tempCelsius = hoodMotor.getDeviceTemp();

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
    PhoenixUtil.tryUntilOk(5, () -> hoodMotor.optimizeBusUtilization(0, 1.0));

    var slot0Configs = new Slot0Configs();
    
    slot0Configs.kP = ShooterConstants.HoodConstants.hoodP;
    slot0Configs.kI = ShooterConstants.HoodConstants.hoodI;
    slot0Configs.kD = ShooterConstants.HoodConstants.hoodD;
    slot0Configs.kS = ShooterConstants.HoodConstants.hoodS;
    slot0Configs.kV = ShooterConstants.HoodConstants.hoodV;

    hoodMotor.getConfigurator().apply(slot0Configs);

  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = hoodMotor.isConnected();
    inputs.encoderConnected = hoodEncoder.isConnected();
    inputs.voltage = hoodMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = hoodMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = hoodEncoder.getVelocity().getValueAsDouble();
    inputs.temperature = hoodMotor.getDeviceTemp().getValueAsDouble();
    inputs.position = hoodEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setPower(double power) {
    hoodMotor.set(power);
  }

  @Override
  public void setPosition(double position) {
    hoodMotor.setControl(hoodRequest.withPosition(position));
  }

  @Override
  public double getPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setVoltage(voltage);
  }

}
