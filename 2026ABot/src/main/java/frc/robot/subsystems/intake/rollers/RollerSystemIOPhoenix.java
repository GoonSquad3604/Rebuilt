package frc.robot.subsystems.intake.rollers;

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
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class RollerSystemIOPhoenix implements RollerSystemIO {

  private final TalonFX rollerSystemMotor;
  private final TalonFXConfiguration motorConfig;
  private final VelocityVoltage rollerSystemRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public RollerSystemIOPhoenix() {

    // motor config:
    rollerSystemMotor = new TalonFX(IntakeConstants.RollerConstants.motorID, Constants.CANBusName);
    rollerSystemRequest = new VelocityVoltage(0).withSlot(0);
    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.Slot0 =
        new Slot0Configs()
            .withKP(IntakeConstants.RollerConstants.P)
            .withKI(IntakeConstants.RollerConstants.I)
            .withKD(IntakeConstants.RollerConstants.D)
            .withKS(IntakeConstants.RollerConstants.S)
            .withKV(IntakeConstants.RollerConstants.V)
            .withKA(IntakeConstants.RollerConstants.A);
    motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

    // apply config
    PhoenixUtil.tryUntilOk(5, () -> rollerSystemMotor.getConfigurator().apply(motorConfig));

    // base status signal
    position = rollerSystemMotor.getPosition();
    velocity = rollerSystemMotor.getVelocity();
    appliedVoltage = rollerSystemMotor.getMotorVoltage();
    supplyCurrent = rollerSystemMotor.getSupplyCurrent();
    torqueCurrent = rollerSystemMotor.getTorqueCurrent();
    tempCelsius = rollerSystemMotor.getDeviceTemp();
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
    PhoenixUtil.tryUntilOk(5, () -> rollerSystemMotor.optimizeBusUtilization(0, 1.0));
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    inputs.motorConnected = rollerSystemMotor.isConnected();
    inputs.voltage = rollerSystemMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = rollerSystemMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = rollerSystemMotor.getVelocity().getValueAsDouble();
    inputs.position = rollerSystemMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocity) {
    rollerSystemMotor.setControl(rollerSystemRequest.withVelocity(velocity));
  }

  @Override
  public void setPower(double power) {
    rollerSystemMotor.set(power);
  }

  @Override
  public void setOpenLoop(double output) {
    rollerSystemMotor.setControl(voltageRequest.withOutput(output));
  }
}
