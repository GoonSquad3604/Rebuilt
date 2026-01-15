package frc.robot.subsystems.Shooter.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  class HoodIOInputs {
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
