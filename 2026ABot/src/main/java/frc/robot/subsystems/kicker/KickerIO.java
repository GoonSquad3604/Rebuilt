package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {

  @AutoLog
  public static class KickerIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double temperature;
    // public double position;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void setPower(double power) {}

  default void setVelocity(double velocity) {}

  default double getVelocity() {
    return 0.0;
  }

  default void setOpenLoop(double output) {}
}
