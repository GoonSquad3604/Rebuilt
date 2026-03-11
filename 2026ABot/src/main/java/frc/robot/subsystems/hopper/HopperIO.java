package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    public boolean motorConnected = false;
    public boolean stowedDetectorConnected = false;
    public double stowedDetectorDistance;
    public boolean stowedDetectorTriggered = false;
    public double voltage;
    public double current;
    public double velocity;
    public double temperature;
    public double position;
  }

  default void updateInputs(HopperIOInputs inputs) {}

  default void setPosition(double position) {}

  default double getPosition() {
    return 0.0;
  }

  default void resetPosition() {}

  default boolean stowedDetectorTriggered() {
    return false;
  }

  default void setPower(double power) {}

  default void setOpenLoop(double output) {}
}
