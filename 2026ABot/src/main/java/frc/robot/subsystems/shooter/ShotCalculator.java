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

  // private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / .02));

  // private double isValid;
  private Rotation2d lastTurretAngle;
  private double adjustment;
  // private double lastHoodPose;
  // private Rotation2d turretAngleRotation2d;
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
      // Rotation2d turretAngleRotation2d,
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
    minDistance = 2;
    maxDistance = 4;
    phaseDelay = 0.05;

    shotFlywheelSpeedMap.put(2.02, 43.0);
    shotFlywheelSpeedMap.put(2.23, 48.0);
    shotFlywheelSpeedMap.put(2.60, 50.0);
    shotFlywheelSpeedMap.put(2.80, 53.0);
    shotFlywheelSpeedMap.put(2.83, 50.0);
    shotFlywheelSpeedMap.put(2.95, 50.0); // ok
    shotFlywheelSpeedMap.put(3.04, 55.0);
    shotFlywheelSpeedMap.put(3.2, 57.0);
    shotFlywheelSpeedMap.put(3.32, 60.0); // ok
    shotFlywheelSpeedMap.put(3.4, 59.0);
    shotFlywheelSpeedMap.put(3.61, 65.0);
    shotFlywheelSpeedMap.put(3.8, 67.0);
    shotFlywheelSpeedMap.put(4.0, 70.0);

    timeOfFlightMap.put(2.02, 0.92);
    timeOfFlightMap.put(2.23, 1.09);
    timeOfFlightMap.put(2.6, 1.0);
    timeOfFlightMap.put(2.8, 1.18);
    timeOfFlightMap.put(2.95, 0.97); // ok
    timeOfFlightMap.put(3.04, 1.19);
    timeOfFlightMap.put(3.2, 1.2);
    timeOfFlightMap.put(3.32, 1.08); // ok
    timeOfFlightMap.put(3.4, 1.3);
    timeOfFlightMap.put(3.61, 1.35);
    timeOfFlightMap.put(3.8, 1.37);
    timeOfFlightMap.put(4.0, 1.4);

    shotHoodAngleMap.put(2.30, 0.1);
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
    // hoodPose = shotHoodAngleMap.get(lookaheadTurretToTargetDistance);
    // if (lastTurretAngle == null) lastTurretAngle = turretAngleRotation2d;
    // if (Double.isNaN(lastHoodPose)) lastHoodPose = hoodPose;

    // lastTurretAngle = turretAngleRotation2d;
    // lastHoodPose = hoodPose;
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
    if (lastTurretAngle == null) lastTurretAngle = Rotation2d.fromDegrees(turretAngle);
    lastTurretAngle = Rotation2d.fromDegrees(turretAngle);
    hoodPose = ShooterConstants.HoodConstants.hoodMaxPos;
    // turretVelocity =
    //     turretAngleFilter.calculate(
    //         Rotation2d.fromDegrees(turretAngle).minus(lastTurretAngle).getRadians() / 0.02);

    // if(turretAngle >= 180){
    //   adjustment = turretAngle/270;
    //   if(adjustment > 1){
    //     adjustment -= 1.0;
    //   }
    // }
    // else{
    //   adjustment = turretAngle/90;
    //   if(adjustment > 1){
    //     adjustment -= 1.0;
    //   }
    // }
    // turretAngle = turretAngle + ShooterConstants.maxAngleAdjustment * adjustment;
    latestParameters =
        new ShootingParameters(
            (lookaheadTurretToTargetDistance >= minDistance
                    && lookaheadTurretToTargetDistance <= maxDistance)
                || RobotState.getInstance().getTarget() != ShooterTarget.HUB,
            // turretAngleRotation2d,
            360 - turretAngle,
            // turretVelocity,
            hoodPose,
            // hoodVelocity,
            shotFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));

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
