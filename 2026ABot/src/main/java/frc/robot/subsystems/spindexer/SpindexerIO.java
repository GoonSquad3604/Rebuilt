package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SpindexerIO {

  @AutoLog
  class SpindexerIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double temperature;
    // public double position;
  }

  default void updateInputs(SpindexerIOInputs inputs) {}

  default void setVelocity(double velocity) {}

  default void setPower(double power) {}

  default void setOpenLoop(double output) {}
}
