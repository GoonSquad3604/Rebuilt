package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.PhoenixUtil;

public class LauncherIOPhoenix implements LauncherIO {

  // motor
  private final TalonFX launcherMotor;
  private final TalonFXConfiguration launcherMotorConfig;
  private final VelocityVoltage launcherRequest;
  private final VoltageOut voltageRequest = new VoltageOut(0);

  // status signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  public LauncherIOPhoenix() {

    launcherMotor =
        new TalonFX(ShooterConstants.LauncherConstants.launcherID, Constants.CANBusName);
    launcherMotorConfig = new TalonFXConfiguration();
    launcherRequest = new VelocityVoltage(0).withSlot(0);

    launcherMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    launcherMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    launcherMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    launcherMotorConfig.CurrentLimits.StatorCurrentLimit = 100;
    launcherMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.LauncherConstants.launcherP)
            .withKI(ShooterConstants.LauncherConstants.launcherI)
            .withKD(ShooterConstants.LauncherConstants.launcherD);
    launcherMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> launcherMotor.getConfigurator().apply(launcherMotorConfig));

    velocity = launcherMotor.getVelocity();
    appliedVoltage = launcherMotor.getMotorVoltage();
    supplyCurrent = launcherMotor.getSupplyCurrent();
    torqueCurrent = launcherMotor.getTorqueCurrent();
    tempCelsius = launcherMotor.getDeviceTemp();

    PhoenixUtil.tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius));
    PhoenixUtil.tryUntilOk(5, () -> launcherMotor.optimizeBusUtilization(0, 1.0));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.LauncherConstants.launcherP;
    slot0Configs.kI = ShooterConstants.LauncherConstants.launcherI;
    slot0Configs.kD = ShooterConstants.LauncherConstants.launcherD;
    slot0Configs.kS = ShooterConstants.LauncherConstants.launcherS;
    slot0Configs.kV = ShooterConstants.LauncherConstants.launcherV;
    slot0Configs.kA = ShooterConstants.LauncherConstants.launcherA;

    launcherMotor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.motorConnected = launcherMotor.isConnected();
    inputs.voltage = launcherMotor.getMotorVoltage().getValueAsDouble();
    inputs.current = launcherMotor.getSupplyCurrent().getValueAsDouble();
    inputs.velocity = launcherMotor.getVelocity().getValueAsDouble();
    inputs.temperature = launcherMotor.getDeviceTemp().getValueAsDouble();
    inputs.position = launcherMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPower(double power) {
    launcherMotor.set(power);
  }

  @Override
  public double getVelocity() {
    return launcherMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void setLauncherOpenLoop(double output) {
    launcherMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setVelocity(double velocity) {
    launcherMotor.setControl(launcherRequest.withVelocity(velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    launcherMotor.setVoltage(voltage);
  }
}
