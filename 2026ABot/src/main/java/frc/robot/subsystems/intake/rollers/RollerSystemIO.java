package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RollerSystemIO {

  @AutoLog
  class RollerSystemIOInputs {
    public boolean motorConnected = false;
    public double voltage;
    public double current;
    public double velocity;
    // public double temperature;
    // public double position;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void setVelocity(double velocity) {}

  default void setPower(double power) {}

  default void setOpenLoop(double output) {}
}
