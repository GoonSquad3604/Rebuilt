package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class KickerIOPhoenix implements KickerIO {

  private final TalonFX kickerMotor;
  private final TalonFXConfiguration motorConfig;
  private final VelocityVoltage kickerRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public KickerIOPhoenix() {

    // motor config:
    kickerMotor = new TalonFX(KickerConstants.motorID);
    kickerRequest = new VelocityVoltage(0).withSlot(0);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(KickerConstants.P)
            .withKI(KickerConstants.I)
            .withKD(KickerConstants.D)
            .withKS(KickerConstants.S)
            .withKV(KickerConstants.V)
            .withKA(KickerConstants.A);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // apply config
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(motorConfig));

    // base status signal
    position = kickerMotor.getPosition();
    velocity = kickerMotor.getVelocity();
    appliedVoltage = kickerMotor.getMotorVoltage();
    supplyCurrent = kickerMotor.getSupplyCurrent();
    torqueCurrent = kickerMotor.getTorqueCurrent();
    tempCelsius = kickerMotor.getDeviceTemp();
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
    PhoenixUtil.tryUntilOk(5, () -> kickerMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.motorConnected = kickerMotor.isConnected();
    inputs.voltage = kickerMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = kickerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = kickerMotor.getVelocity().getValueAsDouble();
    inputs.position = kickerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    kickerMotor.setControl(kickerRequest.withVelocity(velocity));
  }

  @Override
  public void setPower(double power) {
    kickerMotor.set(power);
  }

  @Override
  public void setOpenLoop(double output) {
    kickerMotor.setControl(voltageRequest.withOutput(output));
  }
}
