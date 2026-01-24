package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  default void updateInputs(HoodIOInputs inputs) {}

  @AutoLog
  class HoodIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    public double temperature;
    public double position;
  }

  default void setPower(double power) {}

  default void setPosition(double position) {}
  default void setAngle(double position) {}

  default double getPosition() {return 0.0;}
  default double getAngle() {return 0.0;}

  default void setVoltage(double volts) {}
}
