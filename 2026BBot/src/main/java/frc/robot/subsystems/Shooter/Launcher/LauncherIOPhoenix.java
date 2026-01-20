package frc.robot.subsystems.shooter.launcher;

import com.ctre.phoenix6.hardware.TalonFX;

public class LauncherIOPhoenix implements LauncherIO {

  // private final TalonFX launcherMotor;
  
  public LauncherIOPhoenix() {
    // launchermotor = new motor, woohoo
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {}

  @Override
  public void setPower(double power) {}

  @Override
  public void setRPM(double RPM) {}

  @Override
  public void setVoltage(double voltage) {}
}
