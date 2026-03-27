package frc.robot.subsystems.shooter.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {

  @AutoLog
  public static class LauncherIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double temperature;
    public double position;
  }

  default void updateInputs(LauncherIOInputs inputs) {}

  default void setLauncherOpenLoop(double output) {}

  default void setPower(double power) {}

  default void setVelocity(double velocity) {}

  default void setVoltage(double volts) {}

  default double getVelocity() {
    return 0;
  }
}
