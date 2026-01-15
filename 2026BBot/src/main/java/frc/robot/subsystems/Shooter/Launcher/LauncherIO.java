package frc.robot.subsystems.Shooter.Launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {

  default void updateInputs(LauncherIOInputs inputs) {}

  @AutoLog
  class LauncherIOInputs {
    public boolean isConnected = false;
    public double voltage;
    public double current;
    public double temperature;
    public double RPM;
  }

  default void setPower(double power) {}

  default void setRPM(double RPM) {}

  default void setVoltage(double volts) {}
}
