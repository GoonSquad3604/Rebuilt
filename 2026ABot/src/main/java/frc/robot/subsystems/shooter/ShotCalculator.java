package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.robotToTurret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class ShotCalculator {

  private static ShotCalculator instance;

  private double turretAngle;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean validShootingLocation,
      double turretAngle,
      double hoodPose,
      double primaryFlywheelSpeed) {}
  // double secondaryFlywheelSpeed) {}

  // Cache parameters
  private static ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap shotHoodPositionMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shotPrimaryFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  // private static final InterpolatingDoubleTreeMap shotSecondaryFlywheelSpeedMap =
  //     new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0;
    maxDistance = 3.85;
    phaseDelay = 0.05;

    shotPrimaryFlywheelSpeedMap.put(0.94, 47.0); // min
    shotPrimaryFlywheelSpeedMap.put(1.34, 47.0);
    shotPrimaryFlywheelSpeedMap.put(1.67, 50.0);
    shotPrimaryFlywheelSpeedMap.put(1.8, 52.0);
    shotPrimaryFlywheelSpeedMap.put(2.29, 56.0);
    shotPrimaryFlywheelSpeedMap.put(2.84, 75.0);
    shotPrimaryFlywheelSpeedMap.put(3.15, 85.0);
    shotPrimaryFlywheelSpeedMap.put(3.66, 95.0); // max

    timeOfFlightMap.put(0.94, 1.07); // min
    timeOfFlightMap.put(1.34, 0.98);
    timeOfFlightMap.put(1.67, 1.05);
    timeOfFlightMap.put(1.8, 1.12);
    timeOfFlightMap.put(2.29, 1.16);
    timeOfFlightMap.put(2.84, 1.16);
    timeOfFlightMap.put(3.15, 1.07);
    timeOfFlightMap.put(3.66, 1.05); // max

    shotHoodPositionMap.put(0.94, 0.1); // min
    shotHoodPositionMap.put(1.34, 0.3);
    shotHoodPositionMap.put(1.67, 0.3);
    shotHoodPositionMap.put(1.8, 0.3);
    shotHoodPositionMap.put(2.29, 0.35);
    shotHoodPositionMap.put(2.84, 0.5);
    shotHoodPositionMap.put(3.15, 0.6);
    shotHoodPositionMap.put(3.66, 0.725); // max
  }

  public ShootingParameters getParameters() {

    Translation2d targetPose;
    if (RobotState.getInstance().getTarget() == ShooterTarget.HUB) {
      targetPose = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else if (RobotState.getInstance().getTarget() == ShooterTarget.LEFT_PASS) {
      targetPose = AllianceFlipUtil.apply(ShooterConstants.leftPassPosition);
    } else {
      targetPose = AllianceFlipUtil.apply(ShooterConstants.rightPassPosition);
    }

    Pose2d estimatedPose = RobotState.getInstance().getPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getFieldVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Pose2d turretPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(robotToTurret));
    double turretToTargetDistance = targetPose.getDistance(turretPosition.getTranslation());

    // Calculate field relative turret velocity
    ChassisSpeeds robotVelocity = RobotState.getInstance().getFieldVelocity();
    double robotAngle = estimatedPose.getRotation().getRadians();
    double turretVelocityX =
        robotVelocity.vxMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShooterConstants.robotToTurret.getY() * Math.cos(robotAngle)
                    - ShooterConstants.robotToTurret.getX() * Math.sin(robotAngle));
    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (ShooterConstants.robotToTurret.getX() * Math.cos(robotAngle)
                    - ShooterConstants.robotToTurret.getY() * Math.sin(robotAngle));

    // Account for imparted velocity by robot (turret) to offset
    double timeOfFlight;
    Pose2d lookaheadPose = turretPosition;
    double lookaheadTurretToTargetDistance = turretToTargetDistance;
    for (int i = 0; i < 20; i++) {
      timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
      double offsetX = turretVelocityX * timeOfFlight;
      double offsetY = turretVelocityY * timeOfFlight;

      // Rotation2d offsetRotation = targetPose.minus(lookaheadPose.getTranslation()).getAngle();
      Rotation2d offsetRotation =
          targetPose
              .minus(lookaheadPose.getTranslation())
              .getAngle()
              .minus(
                  Rotation2d.fromRadians(
                      ShooterConstants.robotToTurretLinear
                          * robotVelocity.omegaRadiansPerSecond
                          * phaseDelay));

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              offsetRotation);

      lookaheadTurretToTargetDistance = targetPose.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    turretAngle =
        lookaheadPose
            .getRotation()
            .minus(RobotState.getInstance().getPose().getRotation())
            .getDegrees();
    if (turretAngle > 360) {
      turretAngle -= 360;
    } else if (turretAngle < 0) {
      turretAngle += 360;
    }

    latestParameters =
        new ShootingParameters(
            (lookaheadTurretToTargetDistance >= minDistance
                    && lookaheadTurretToTargetDistance <= maxDistance)
                || RobotState.getInstance().getTarget() != ShooterTarget.HUB,
            360 - turretAngle,
            shotHoodPositionMap.get(lookaheadTurretToTargetDistance),
            shotPrimaryFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/Parameters", latestParameters);
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/LookaheadPose", lookaheadPose);
    // Logger.recordOutput(
    //     "Subsystems/Shooter/ShotCalculator/TurretToTargetDistance",
    //     lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
