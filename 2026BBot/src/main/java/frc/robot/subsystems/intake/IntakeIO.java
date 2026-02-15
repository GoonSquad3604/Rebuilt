package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {

    // intaking motor variables
    public double intakeMotorSpeed;
    public double intakeMotorVoltage;
    public double intakeMotorCurrent;
    public double intakeMotorTemp;

    // logging
    public boolean intakeMotorConnected;
  }

  // intaking functions
  default void setVoltage(double voltage) {}

  default void setPower(double power) {}

  default void updateInputs(IntakeIOInputs inputs) {}
}
