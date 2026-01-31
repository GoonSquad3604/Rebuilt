// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.util.RobotState;

public class Turret extends SubsystemBase {

  private SparkMax turretMotor;
  private RelativeEncoder turretEncoder;

  private double wantedAngle;

  // private ShootingParameters shootingParameters;

  public Turret() {

    turretMotor = new SparkMax(7, MotorType.kBrushless);
    turretEncoder = turretMotor.getEncoder();

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig.idleMode(IdleMode.kBrake);
    turretConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.turretP, ShooterConstants.turretI, ShooterConstants.turretD);

    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void turnCounterClockwise() {
    turretMotor.set(.3);
  }

  public void turnClockwise() {
    turretMotor.set(-.3);
  }

  public void stopTurret() {
    turretMotor.set(0);
  }

  public void setPosition(double position) {

    if (position > ShooterConstants.maxEncoderPos) {
      position = ShooterConstants.maxEncoderPos;
    } else if (position < ShooterConstants.minEncoderPos) {
      position = ShooterConstants.minEncoderPos;
    }
    turretMotor
        .getClosedLoopController()
        .setSetpoint(
            position,
            SparkFlex.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ShooterConstants.turretFF);
  }

  public void setAngle(double angle) {
    if (angle > ShooterConstants.maxAngle) {
      angle = ShooterConstants.maxAngle;
    } else if (angle < ShooterConstants.minAngle) {
      angle = ShooterConstants.minAngle;
    }
    turretMotor
        .getClosedLoopController()
        .setSetpoint(
            getEncoderValueFromDegrees(angle),
            SparkFlex.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ShooterConstants.turretFF);
  }

  private double getEncoderValueFromDegrees(double degrees) {
    double result = (ShooterConstants.maxEncoderPos / ShooterConstants.maxAngle) * degrees;
    return result;
  }

  private double getDegreesFromRotations(double rotations) {
    double result = (ShooterConstants.maxAngle / ShooterConstants.maxEncoderPos) * rotations;
    return result;
  }

  public void zeroEncoder() {
    turretEncoder.setPosition(0);
  }

  public double getWantedAngle() {
    return wantedAngle;
  }

  public double trackHub() {
    Translation2d hubPos = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Translation2d robotPos = RobotState.getInstance().getPose().getTranslation();
    Translation2d turretPos = robotPos.plus(ShooterConstants.robotToTurret.getTranslation());

    double targetX = hubPos.getX() - turretPos.getX();
    double targetY = hubPos.getY() - turretPos.getY();
    Translation2d targetPosition = new Translation2d(targetX, targetY);

    double distance = targetPosition.getNorm();
    double idealSpeed = getShooterSpeedForDistance(distance);

    Translation2d targetVector = targetPosition.div(distance).times(idealSpeed);

    Translation2d robotVelocity =
        new Translation2d(
            RobotState.getInstance().getFieldVelocity().vxMetersPerSecond,
            RobotState.getInstance().getFieldVelocity().vyMetersPerSecond);

    Translation2d shotVector = targetVector.minus(robotVelocity);

    double turretAngle =
        shotVector.getAngle().getDegrees()
            - RobotState.getInstance().getPose().getRotation().getDegrees()
            - 90;

    if (turretAngle > 360) {
      turretAngle -= 360;
    } else if (turretAngle < 0) {
      turretAngle += 360;
    }

    return turretAngle;

    // setAngle(shotVelocity.getAngle().getDegrees());
    // return (shotVelocity.getAngle().getDegrees() + 180)
    //     + (RobotState.getInstance().getPose().getRotation().getDegrees() + 180);
  }

  public double getShooterSpeedForDistance(double distance) {
    return .5;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("turret encoder", turretEncoder.getPosition());
    wantedAngle = SmartDashboard.getNumber("wantedAngle", 0);
    SmartDashboard.putNumber("wantedAngle", wantedAngle);
    SmartDashboard.putNumber("turret angle", getDegreesFromRotations(turretEncoder.getPosition()));

    // shootingParameters = ShotCalculator.getInstance().getParameters();

    SmartDashboard.putNumber("calculated turret angle", trackHub());
  }
}
