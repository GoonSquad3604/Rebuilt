package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.hardware.TalonFX;

public class TurretIOPhoenix implements TurretIO {

  // private final TalonFX turretMotor;

  public TurretIOPhoenix() {
    // turretmotor = new motor, woohoo
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {}

  @Override
  public void setPower(double power) {}

  @Override
  public void setPosition(double position) {}

  @Override
  public void setVoltage(double voltage) {}
}
