package frc.robot.subsystems.shooter.turret;

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

public class TurretIOPhoenix implements TurretIO {

  // motor
  private final TalonFX turretMotor;
  private final PositionVoltage turretRequest;
  private final TalonFXConfiguration turretMotorConfig;

  // encoder
  private final CANcoder turretEncoder;
  private final CANcoderConfiguration turretEncoderConfig;

  // status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public TurretIOPhoenix() {

    turretMotor = new TalonFX(ShooterConstants.TurretConstants.turretID);
    turretRequest = new PositionVoltage(0);

    turretEncoder = new CANcoder(ShooterConstants.TurretConstants.turretEncoderID);
    turretEncoderConfig = new CANcoderConfiguration();

    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID =
        ShooterConstants.TurretConstants.turretEncoderID;
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
    turretMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.TurretConstants.turretP)
            .withKI(ShooterConstants.TurretConstants.turretI)
            .withKD(ShooterConstants.TurretConstants.turretD);
    turretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretMotorConfig));

    position = turretMotor.getPosition();
    velocity = turretMotor.getVelocity();
    appliedVoltage = turretMotor.getMotorVoltage();
    supplyCurrent = turretMotor.getSupplyCurrent();
    torqueCurrent = turretMotor.getTorqueCurrent();
    tempCelsius = turretMotor.getDeviceTemp();

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
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.optimizeBusUtilization(0, 1.0));

    var slot0Configs = new Slot0Configs();

    slot0Configs.kP = ShooterConstants.TurretConstants.turretP;
    slot0Configs.kI = ShooterConstants.TurretConstants.turretI;
    slot0Configs.kD = ShooterConstants.TurretConstants.turretD;
    slot0Configs.kS = ShooterConstants.TurretConstants.turretS;
    slot0Configs.kV = ShooterConstants.TurretConstants.turretV;

    turretMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = turretMotor.isConnected();
    inputs.encoderConnected = turretEncoder.isConnected();
    inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = turretMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = turretEncoder.getVelocity().getValueAsDouble();
    inputs.temperature = turretMotor.getDeviceTemp().getValueAsDouble();
    inputs.position = turretEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setPower(double power) {
    turretMotor.set(power);
  }

  @Override
  public void setPosition(double position) {
    turretMotor.setControl(turretRequest.withPosition(position));
  }

  @Override
  public void setAngle(double angle) {
    turretMotor.setControl(turretRequest.withPosition(convertAngleToRotations(angle)));
  }

  @Override
  public double getPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getAngle() {
    return convertRotationsToAngle(turretMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  private double convertAngleToRotations(double angle) {
    return 0.0;
  }

  private double convertRotationsToAngle(double rotations) {
    return 0.0;
  }
}
