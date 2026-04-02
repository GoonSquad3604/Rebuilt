package frc.robot.subsystems.shooter.primaryLauncher;

import org.littletonrobotics.junction.AutoLog;

public interface PrimaryLauncherIO {

  @AutoLog
  public static class PrimaryLauncherIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double primaryTemperature;
    public double position;
  }

  default void updateInputs(PrimaryLauncherIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPower(double power) {}

  default void setVelocity(double velocity) {}

  default void setVoltage(double volts) {}

  default double getVelocity() {
    return 0;
  }
}
