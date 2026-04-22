package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Degrees;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

public class TurretIOPhoenix implements TurretIO {

  // motor
  private final TalonFX turretMotor;
  // private final MotionMagicVoltage turretRequest = new MotionMagicVoltage(0.0);
  private final PositionVoltage turretRequest;

  private final TalonFXConfiguration turretMotorConfig;

  // encoder
  private final CANcoder turretEncoder;
  private final CANcoderConfiguration turretEncoderConfig;

  private final VoltageOut voltageRequest;

  // status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public TurretIOPhoenix() {

    // motor config
    turretMotor = new TalonFX(ShooterConstants.TurretConstants.motorID, Constants.CANBusName);
    turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.ClosedLoopGeneral.ContinuousWrap = false;
    turretMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID = ShooterConstants.TurretConstants.encoderID;
    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    // encoder config
    turretEncoder = new CANcoder(ShooterConstants.TurretConstants.encoderID, Constants.CANBusName);
    turretEncoderConfig = new CANcoderConfiguration();

    turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    turretEncoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.TurretConstants.encoderOffset;
    turretEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;

    // pid configs

    voltageRequest = new VoltageOut(0);
    turretRequest = new PositionVoltage(0.0);
    // turretMotorConfig.MotionMagic.MotionMagicAcceleration =
    //     ShooterConstants.TurretConstants.turretAcceleration;
    // turretMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
    //     ShooterConstants.TurretConstants.turretVelocity;

    turretMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.TurretConstants.P)
            .withKI(ShooterConstants.TurretConstants.I)
            .withKD(ShooterConstants.TurretConstants.D)
            .withKS(ShooterConstants.TurretConstants.S)
            .withKV(ShooterConstants.TurretConstants.V)
            .withKA(ShooterConstants.TurretConstants.A);
    turretMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        ShooterConstants.TurretConstants.rampRate;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> turretEncoder.getConfigurator().apply(turretEncoderConfig));
    PhoenixUtil.tryUntilOk(5, () -> turretMotor.getConfigurator().apply(turretMotorConfig));

    position = turretEncoder.getAbsolutePosition();
    velocity = turretEncoder.getVelocity();
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
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = turretMotor.isConnected();
    inputs.encoderConnected = turretEncoder.isConnected();
    inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = turretMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.velocity = turretEncoder.getVelocity().getValueAsDouble();
    // inputs.temperature = turretMotor.getDeviceTemp().getValueAsDouble();
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
    turretMotor.setControl(
        turretRequest
            .withPosition(
                MathUtil.clamp(
                    position,
                    ShooterConstants.TurretConstants.minEncoderPosition,
                    ShooterConstants.TurretConstants.maxEncoderPosition))
            .withEnableFOC(true));
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
