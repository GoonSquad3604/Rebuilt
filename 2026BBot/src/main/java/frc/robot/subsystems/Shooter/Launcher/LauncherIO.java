package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {

  default void updateInputs(LauncherIOInputs inputs) {}

  @AutoLog
  class LauncherIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double RPM;
    public double temperature;
  }

  default void setPower(double power) {}

  default void setRPM(double RPM) {}

  default void setVoltage(double volts) {}
}
