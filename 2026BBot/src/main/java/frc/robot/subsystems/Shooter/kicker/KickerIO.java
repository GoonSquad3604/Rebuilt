package frc.robot.subsystems.shooter.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {

  @AutoLog
  public static class KickerIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double velocity;
    public double current;
    public double temperature;
    public double position;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void setKickerOpenLoop(double output) {}

  public void setPower(double power);

  public void setVelocity(double velocity);

  public void setVoltage(double volts);
}
