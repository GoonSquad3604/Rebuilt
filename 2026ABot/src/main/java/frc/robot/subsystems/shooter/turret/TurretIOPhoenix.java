package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.Constants;
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

  private final VoltageOut voltageRequest = new VoltageOut(0);

  // status signals
  // private final StatusSignal<Angle> position;
  // private final StatusSignal<AngularVelocity> velocity;
  // private final StatusSignal<Voltage> appliedVoltage;
  // private final StatusSignal<Current> supplyCurrent;
  // private final StatusSignal<Current> torqueCurrent;
  // private final StatusSignal<Temperature> tempCelsius;

  public TurretIOPhoenix() {

    turretMotor = new TalonFX(ShooterConstants.TurretConstants.turretID, Constants.CANBusName);
    turretRequest = new PositionVoltage(0);

    turretEncoder =
        new CANcoder(ShooterConstants.TurretConstants.turretEncoderID, Constants.CANBusName);
    turretEncoderConfig = new CANcoderConfiguration();

    turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    turretEncoderConfig.MagnetSensor.MagnetOffset = 0.025;
    turretEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID =
        ShooterConstants.TurretConstants.turretEncoderID;
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = .999;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    turretMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.TurretConstants.turretP)
            .withKI(ShooterConstants.TurretConstants.turretI)
            .withKD(ShooterConstants.TurretConstants.turretD)
            .withKS(ShooterConstants.TurretConstants.turretS)
            .withKV(ShooterConstants.TurretConstants.turretV);
    turretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
    PhoenixUtil.tryUntilOk(5, () -> turretEncoder.getConfigurator().apply(turretEncoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretMotorConfig));

    // position = turretEncoder.getAbsolutePosition();
    // velocity = turretEncoder.getVelocity();
    // appliedVoltage = turretMotor.getMotorVoltage();
    // supplyCurrent = turretMotor.getSupplyCurrent();
    // torqueCurrent = turretMotor.getTorqueCurrent();
    // tempCelsius = turretMotor.getDeviceTemp();

    // PhoenixUtil.tryUntilOk(
    //     5,
    //     () ->
    //         BaseStatusSignal.setUpdateFrequencyForAll(
    //             50.0,
    //             position,
    //             velocity,
    //             appliedVoltage,
    //             supplyCurrent,
    //             torqueCurrent,
    //             tempCelsius));
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.optimizeBusUtilization(0, 1.0));
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
    inputs.angle = turretEncoder.getAbsolutePosition().getValue().in(Degrees);
    inputs.motorPosition = turretMotor.getPosition().getValueAsDouble();
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

  // my favorite angle is 210 -lucas
  @Override
  public double getAngle() {
    return turretEncoder.getAbsolutePosition().getValue().in(Degrees);
  }

  @Override
  public void setVoltage(double voltage) {
    turretMotor.setVoltage(voltage);
  }

  private double convertAngleToRotations(double angle) {
    double newValue = (angle) / 360;
    if (newValue > ShooterConstants.TurretConstants.maxEncoderPosition) {
      newValue = ShooterConstants.TurretConstants.maxEncoderPosition;
    } else if (newValue < ShooterConstants.TurretConstants.minEncoderPosition) {
      newValue = ShooterConstants.TurretConstants.minEncoderPosition;
    }
    return newValue;
  }

  @Override
  public void setTurretOpenLoop(double output) {
    turretMotor.setControl(voltageRequest.withOutput(output));
  }
}
