package frc.robot.subsystems.Shooter.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  default void updateInputs(TurretIOInputs inputs) {}

  @AutoLog
  class TurretIOInputs {
    public boolean isConnected = false;
    public double voltage;
    public double current;
    public double temperature;
    public double position;
  }

  default void setPower(double power) {}

  default void setPosition(double position) {}

  default void setVoltage(double volts) {}
}
