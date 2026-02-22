// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;

public class ShotCalculator {
  private static ShotCalculator instance;

  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / ShooterConstants.loopPeriodSecs));
  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / ShooterConstants.loopPeriodSecs));

  private double lastTurretAngle;
  private double lastHoodAngle;
  private double hoodAngle = Double.NaN;
  private double turretVelocity;
  private double hoodVelocity;
  private double hoodPos;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      double turretAngle, double idealVelocity, double hoodPos, double turretVelocity) {}

  // Cache parameters
  private static ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.02;

    shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    shotHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    shotHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    shotHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    shotHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    shotHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    shotFlywheelSpeedMap.put(1.34, 210.0);
    shotFlywheelSpeedMap.put(1.78, 220.0);
    shotFlywheelSpeedMap.put(2.17, 220.0);
    shotFlywheelSpeedMap.put(2.81, 230.0);
    shotFlywheelSpeedMap.put(3.82, 250.0);
    shotFlywheelSpeedMap.put(4.09, 255.0);
    shotFlywheelSpeedMap.put(4.40, 260.0);
    shotFlywheelSpeedMap.put(4.77, 265.0);
    shotFlywheelSpeedMap.put(5.57, 275.0);
    shotFlywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootingParameters getParameters() {
    Translation2d pos;
    if (RobotState.getInstance().getTarget() == ShooterTarget.HUB) {
      pos = FieldConstants.Hub.topCenterPoint.toTranslation2d();
      hoodPos = 0.0;
    } else if (RobotState.getInstance().getTarget() == ShooterTarget.LEFT_PASS) {
      pos = new Translation2d(2.203, 6.125);
      hoodPos = 0.7;
    } else {
      pos = new Translation2d(2.203, 2.125);
      hoodPos = 0.7;
    }

    Translation2d hubPos = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    Pose2d robotPos = RobotState.getInstance().getPose();
    Pose2d turretPos = robotPos.plus(ShooterConstants.robotToTurret);

    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();

    robotPos =
        robotPos.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

    double targetX = pos.getX() - turretPos.getX();
    double targetY = pos.getY() - turretPos.getY();
    Translation2d targetPosition = new Translation2d(targetX, targetY);

    double distance = targetPosition.getNorm();
    // double idealSpeed = getShooterSpeedForDistance(distance);
    double idealVelocity = .5; // 60;
    Translation2d targetVector = targetPosition.div(distance).times(idealVelocity);

    Translation2d shotVector =
        targetVector.minus(
            new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond));

    double turretAngle =
        shotVector.getAngle().getDegrees()
            - RobotState.getInstance().getPose().getRotation().getDegrees();

    if (turretAngle > 360) {
      turretAngle -= 360;
    } else if (turretAngle < 0) {
      turretAngle += 360;
    }

    if (Double.isNaN(lastTurretAngle)) lastTurretAngle = turretAngle;
    double turretVelocity =
        turretAngleFilter.calculate(
            (turretAngle - lastTurretAngle) / ShooterConstants.loopPeriodSecs);
    lastTurretAngle = turretAngle;
    latestParameters = new ShootingParameters(turretAngle, idealVelocity, hoodPos, turretVelocity);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
