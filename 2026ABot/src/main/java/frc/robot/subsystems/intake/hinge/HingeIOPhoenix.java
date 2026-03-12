package frc.robot.subsystems.intake.hinge;

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
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class HingeIOPhoenix implements HingeIO {

  private final TalonFX hingeMotor;
  private final CANcoder hingeEncoder;

  private final TalonFXConfiguration motorConfig;
  private final CANcoderConfiguration encoderConfig;

  private final PositionVoltage hingeRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // private final StatusSignal<Angle> position;
  // private final StatusSignal<AngularVelocity> velocity;
  // private final StatusSignal<Voltage> appliedVoltage;
  // private final StatusSignal<Current> supplyCurrent;
  // private final StatusSignal<Current> torqueCurrent;
  // private final StatusSignal<Temperature> tempCelsius;

  public HingeIOPhoenix() {

    // motor config:
    hingeMotor = new TalonFX(IntakeConstants.HingeConstants.motorID, Constants.CANBusName);
    hingeEncoder = new CANcoder(IntakeConstants.HingeConstants.encoderID, Constants.CANBusName);
    hingeRequest = new PositionVoltage(0).withSlot(0);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.FeedbackRemoteSensorID = IntakeConstants.HingeConstants.encoderID;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.HingeConstants.P)
            .withKI(IntakeConstants.HingeConstants.I)
            .withKD(IntakeConstants.HingeConstants.D)
            .withKS(IntakeConstants.HingeConstants.S)
            .withKV(IntakeConstants.HingeConstants.V)
            .withKA(IntakeConstants.HingeConstants.A)
            .withKG(IntakeConstants.HingeConstants.G);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = .75;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> hingeMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> hingeEncoder.getConfigurator().apply(encoderConfig));

    // base status signal
    // position = hingeEncoder.getPosition();
    // velocity = hingeEncoder.getVelocity();
    // appliedVoltage = hingeMotor.getMotorVoltage();
    // supplyCurrent = hingeMotor.getSupplyCurrent();
    // torqueCurrent = hingeMotor.getTorqueCurrent();
    // tempCelsius = hingeMotor.getDeviceTemp();
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

    // optimize bus utilization
    PhoenixUtil.tryUntilOk(5, () -> hingeMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(HingeIOInputs inputs) {
    inputs.motorConnected = hingeMotor.isConnected();
    inputs.encoderConnected = hingeEncoder.isConnected();
    inputs.voltage = hingeMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = hingeMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = hingeEncoder.getVelocity().getValueAsDouble();
    inputs.position = hingeEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    if (position == IntakeConstants.HingeConstants.stowedPosition) {
      hingeMotor.setControl(hingeRequest.withPosition(position));
    } else {
      hingeMotor.setControl(hingeRequest.withPosition(position));
    }
  }

  @Override
  public double getPosition() {
    return hingeMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPower(double power) {
    hingeMotor.set(power);
  }

  @Override
  public void setOpenLoop(double output) {
    hingeMotor.setControl(voltageRequest.withOutput(output));
  }
}
