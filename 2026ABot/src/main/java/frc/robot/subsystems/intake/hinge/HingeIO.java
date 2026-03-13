package frc.robot.subsystems.intake.hinge;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HingeIO {

  @AutoLog
  class HingeIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;
    public double voltage;
    public double current;
    // public double velocity;
    // public double temperature;
    public double position;
  }

  default void updateInputs(HingeIOInputs inputs) {}

  default void setPosition(double position) {}

  default double getPosition() {
    return 0.0;
  }

  default void setPower(double power) {}

  default void setOpenLoop(double output) {}
}
