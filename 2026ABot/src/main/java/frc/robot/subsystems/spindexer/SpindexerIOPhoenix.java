package frc.robot.subsystems.spindexer;

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
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class SpindexerIOPhoenix implements SpindexerIO {

  private final TalonFX spindexerMotor;

  private final TalonFXConfiguration motorConfig;

  private final VelocityVoltage spindexerRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public SpindexerIOPhoenix() {

    // motor config:
    spindexerMotor = new TalonFX(SpindexerConstants.motorID, Constants.CANBusName);
    spindexerRequest = new VelocityVoltage(0).withSlot(0);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(SpindexerConstants.P)
            .withKI(SpindexerConstants.I)
            .withKD(SpindexerConstants.D)
            .withKS(SpindexerConstants.S)
            .withKV(SpindexerConstants.V)
            .withKA(SpindexerConstants.A);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // apply configs
    PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(motorConfig));

    // base status signal
    position = spindexerMotor.getPosition();
    velocity = spindexerMotor.getVelocity();
    appliedVoltage = spindexerMotor.getMotorVoltage();
    supplyCurrent = spindexerMotor.getSupplyCurrent();
    torqueCurrent = spindexerMotor.getTorqueCurrent();
    tempCelsius = spindexerMotor.getDeviceTemp();
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
    PhoenixUtil.tryUntilOk(5, () -> spindexerMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.motorConnected = spindexerMotor.isConnected();
    inputs.voltage = spindexerMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = spindexerMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = spindexerMotor.getVelocity().getValueAsDouble();
    inputs.position = spindexerMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    spindexerMotor.setControl(spindexerRequest.withVelocity(velocity));
  }

  @Override
  public void setPower(double power) {
    spindexerMotor.set(power);
  }

  @Override
  public void setOpenLoop(double output) {
    spindexerMotor.setControl(voltageRequest.withOutput(output));
  }
}
