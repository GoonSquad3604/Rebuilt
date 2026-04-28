package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public boolean outerMotorConnected = false;
    public boolean outerEncoderConnected = false;
    public double outerVoltage;
    public double outerCurrent;
    public double outerVelocity;
    // public double outerTemperature;
    public double outerPosition;

    public boolean innerMotorConnected = false;
    public boolean innerEncoderConnected = false;
    public double innerVoltage;
    public double innerCurrent;
    public double innerVelocity;
    // public double innerTemperature;
    public double innerPosition;

    // public boolean leftClimbRangeConnected;
    // public boolean rightClimbRangeConnected;
    public boolean centerClimbRangeConnected;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setOuterPosition(double position) {}

  default void setOuterPower(double power) {}

  default void setOuterOpenLoop(double output) {}

  default double getOuterPosition() {
    return 0.0;
  }

  default void setInnerPosition(double position) {}

  default void setInnerPower(double power) {}

  default void setInnerOpenLoop(double output) {}

  default double getInnerPosition() {
    return 0.0;
  }

  // default boolean leftClimbDetected() {
  //   return false;
  // }

  // default boolean rightClimbDetected() {
  //   return false;
  // }

  default boolean centerClimbDetected() {
    return false;
  }
}
