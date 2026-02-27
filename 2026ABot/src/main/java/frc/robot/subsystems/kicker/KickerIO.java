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
    public double temperature;
    public double position;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  public void setPower(double power);

  public void setVelocity(double velocity);

  default void setOpenLoop(double output) {}
}
