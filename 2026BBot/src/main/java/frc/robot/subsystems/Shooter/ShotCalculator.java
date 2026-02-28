package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

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
import org.littletonrobotics.junction.Logger;

public class ShotCalculator {
  private static ShotCalculator instance;

  private Rotation2d lastTurretAngle;
  private double lastHoodPose;
  private Rotation2d turretAngleRotation2d;
  private double turretAngle;
  private double hoodPose = Double.NaN;
  // private double turretVelocity;
  // private double hoodVelocity;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean isValid,
      Rotation2d turretAngleRotation2d,
      double turretAngle,
      // double turretVelocity,
      double hoodPose,
      // double hoodVelocity,
      double flywheelSpeed) {}

  // Cache parameters
  private static ShootingParameters latestParameters = null;

  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static final InterpolatingDoubleTreeMap shotHoodAngleMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    phaseDelay = 0.02;

    shotHoodAngleMap.put(1.34, 0.0);
    shotHoodAngleMap.put(1.78, 0.1);
    shotHoodAngleMap.put(2.17, 0.2);
    shotHoodAngleMap.put(2.81, 0.3);
    shotHoodAngleMap.put(3.82, 0.4);
    shotHoodAngleMap.put(4.09, 0.425);
    shotHoodAngleMap.put(4.40, 0.475);
    shotHoodAngleMap.put(4.77, 0.5);
    shotHoodAngleMap.put(5.57, 0.6);
    shotHoodAngleMap.put(5.60, 0.7);

    shotFlywheelSpeedMap.put(1.34, 40.0);
    shotFlywheelSpeedMap.put(1.78, 42.0);
    shotFlywheelSpeedMap.put(2.17, 44.0);
    shotFlywheelSpeedMap.put(2.81, 46.0);
    shotFlywheelSpeedMap.put(3.82, 48.0);
    shotFlywheelSpeedMap.put(4.09, 50.0);
    shotFlywheelSpeedMap.put(4.40, 52.0);
    shotFlywheelSpeedMap.put(4.77, 54.0);
    shotFlywheelSpeedMap.put(5.57, 56.0);
    shotFlywheelSpeedMap.put(5.60, 58.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  public ShootingParameters getParameters() {

    Translation2d targetPose;
    if (RobotState.getInstance().getTarget() == ShooterTarget.HUB) {
      targetPose = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    } else if (RobotState.getInstance().getTarget() == ShooterTarget.LEFT_PASS) {
      targetPose = AllianceFlipUtil.apply(new Translation2d(2.203, 6.125));
    } else {
      targetPose = AllianceFlipUtil.apply(new Translation2d(2.203, 2.125));
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

      turretAngleRotation2d = targetPose.minus(lookaheadPose.getTranslation()).getAngle();

      lookaheadPose =
          new Pose2d(
              turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              turretAngleRotation2d);

      lookaheadTurretToTargetDistance = targetPose.getDistance(lookaheadPose.getTranslation());
    }

    // Calculate parameters accounted for imparted velocity
    hoodPose = shotHoodAngleMap.get(lookaheadTurretToTargetDistance);
    if (lastTurretAngle == null) lastTurretAngle = turretAngleRotation2d;
    if (Double.isNaN(lastHoodPose)) lastHoodPose = hoodPose;

    lastTurretAngle = turretAngleRotation2d;
    lastHoodPose = hoodPose;
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
            lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance,
            turretAngleRotation2d,
            turretAngle,
            // turretVelocity,
            hoodPose,
            // hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    Logger.recordOutput("Subsystems/Shooter/ShotCalculator/Parameters", latestParameters);
    Logger.recordOutput("Subsystems/Shooter/ShotCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput(
        "Subsystems/Shooter/ShotCalculator/TurretToTargetDistance",
        lookaheadTurretToTargetDistance);

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
