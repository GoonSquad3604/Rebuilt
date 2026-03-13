package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
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

/** Add your docs here. */
public class HopperIOPhoenix implements HopperIO {

  private final TalonFX hopperMotor;
  private final CANrange stowedDetector;

  private final TalonFXConfiguration motorConfig;
  private final CANrangeConfiguration stowedDetectorConfig;

  private final PositionVoltage hopperRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public HopperIOPhoenix() {

    // motor config:
    hopperMotor = new TalonFX(HopperConstants.motorID, Constants.CANBusName);
    hopperRequest = new PositionVoltage(0).withSlot(0);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(HopperConstants.P)
            .withKI(HopperConstants.I)
            .withKD(HopperConstants.D)
            .withKS(HopperConstants.S)
            .withKV(HopperConstants.V)
            .withKA(HopperConstants.A);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 2;

    // CAN Range config (stowed detector)
    stowedDetector = new CANrange(HopperConstants.stowedDetectorID, Constants.CANBusName);
    stowedDetectorConfig = new CANrangeConfiguration();

    stowedDetectorConfig.ProximityParams.MinSignalStrengthForValidMeasurement =
        HopperConstants.stowedDetectorTriggerDistance;
    stowedDetectorConfig.FovParams.FOVCenterX = 0;
    stowedDetectorConfig.FovParams.FOVCenterY = 0;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> stowedDetector.getConfigurator().apply(stowedDetectorConfig));

    // base status signal
    position = hopperMotor.getPosition();
    velocity = hopperMotor.getVelocity();
    appliedVoltage = hopperMotor.getMotorVoltage();
    supplyCurrent = hopperMotor.getSupplyCurrent();
    torqueCurrent = hopperMotor.getTorqueCurrent();
    tempCelsius = hopperMotor.getDeviceTemp();
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

    // optimize bus utilization
    PhoenixUtil.tryUntilOk(5, () -> hopperMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.motorConnected = hopperMotor.isConnected();
    inputs.stowedDetectorConnected = stowedDetector.isConnected();
    // inputs.stowedDetectorDistance = stowedDetector.getDistance().getValueAsDouble();
    inputs.stowedDetectorTriggered = stowedDetector.getIsDetected().getValue().booleanValue();
    inputs.voltage = hopperMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = hopperMotor.getSupplyCurrent().getValueAsDouble();
    // inputs.velocity = hopperMotor.getVelocity().getValueAsDouble();
    inputs.position = hopperMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPosition(double position) {
    hopperMotor.setControl(hopperRequest.withPosition(position));
  }

  @Override
  public double getPosition() {
    return hopperMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setEncoderPosition(double position) {
    hopperMotor.setPosition(position);
  }

  @Override
  public boolean stowedDetectorTriggered() {
    if (stowedDetector.getIsDetected().getValue().booleanValue()) resetPosition();
    return stowedDetector.getIsDetected().getValue().booleanValue();
  }

  @Override
  public void setPower(double power) {
    hopperMotor.set(power);
  }

  @Override
  public void setOpenLoop(double output) {
    hopperMotor.setControl(voltageRequest.withOutput(output));
  }
}
