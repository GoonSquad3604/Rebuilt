package frc.robot.subsystems.shooter.secondaryLauncher;

import org.littletonrobotics.junction.AutoLog;

public interface SecondaryLauncherIO {

  @AutoLog
  public static class SecondaryLauncherIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double temperature;
    public double position;
  }

  default void updateInputs(SecondaryLauncherIOInputs inputs) {}

  default void setOpenLoop(double output) {}

  default void setPower(double power) {}

  default void setVelocity(double velocity) {}

  default void setVoltage(double volts) {}

  default double getVelocity() {
    return 0;
  }
}
