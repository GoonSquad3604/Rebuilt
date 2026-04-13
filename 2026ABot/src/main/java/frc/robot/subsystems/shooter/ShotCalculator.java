package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.robotToTurret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class ShotCalculator {

  private static ShotCalculator instance;
  private boolean isValid;
  private double turretAngle;
  private double hoodPosition;
  private boolean isPassing;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean validShootingLocation,
      double turretAngle,
      double hoodPosition,
      double flywheelVelocity) {}

  // Cache parameters
  private static ShootingParameters latestParameters = null;
  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;

  private static final InterpolatingDoubleTreeMap shotFlywheelVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shotHoodPositionMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0;
    maxDistance = 3604;
    phaseDelay = 0.1125;

    shotFlywheelVelocityMap.put(1.751, 45.0); // hub
    shotFlywheelVelocityMap.put(2.127, 48.5);
    shotFlywheelVelocityMap.put(2.813, 50.0);
    shotFlywheelVelocityMap.put(3.023, 53.0);
    shotFlywheelVelocityMap.put(3.463, 55.0);
    shotFlywheelVelocityMap.put(4.336, 63.0);
    shotFlywheelVelocityMap.put(4.743, 66.0);

    shotHoodPositionMap.put(1.751, 0.07); // hub
    shotHoodPositionMap.put(2.127, 0.475);
    shotHoodPositionMap.put(2.813, 0.475);
    shotHoodPositionMap.put(3.023, 0.5);
    shotHoodPositionMap.put(3.463, 0.525);
    shotHoodPositionMap.put(4.336, 0.59);
    shotHoodPositionMap.put(4.743, 0.6);

    timeOfFlightMap.put(1.751, 1.0); // hub
    timeOfFlightMap.put(2.127, 1.0);
    timeOfFlightMap.put(2.813, 1.15);
    timeOfFlightMap.put(3.023, 1.17);
    timeOfFlightMap.put(3.463, 1.25);
    timeOfFlightMap.put(4.336, 1.28);
    timeOfFlightMap.put(4.743, 1.3);
  }

  public ShootingParameters getParameters() {

    Translation2d targetPose;
    if (RobotState.getInstance().getTarget() == ShooterTarget.HUB) {
      targetPose = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
      isPassing = false;
    } else if (RobotState.getInstance().getTarget() == ShooterTarget.LEFT_PASS) {
      targetPose = AllianceFlipUtil.apply(ShooterConstants.leftPassPosition);
      isPassing = true;
    } else {
      targetPose = AllianceFlipUtil.apply(ShooterConstants.rightPassPosition);
      isPassing = true;
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

    /* Calculate parameters accounted for predicted position */

    // distance deadzone
    isValid =
        (lookaheadTurretToTargetDistance >= minDistance
                && lookaheadTurretToTargetDistance <= maxDistance)
            || RobotState.getInstance().getTarget() != ShooterTarget.HUB;

    // tower deadzone
    isValid =
        isValid
            && !(RobotState.getInstance().isUnderTower(lookaheadPose))
            && !(RobotState.getInstance().isBehindHub(lookaheadPose))
            && !(RobotState.getInstance().nearTrench());

    // turret angle
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

    turretAngle = 360 - turretAngle;

    // hood position
    hoodPosition = shotHoodPositionMap.get(lookaheadTurretToTargetDistance);

    double flywheelVelocity = shotFlywheelVelocityMap.get(lookaheadTurretToTargetDistance);

    // if (isPassing) {
    //   hoodPosition = ShooterConstants.HoodConstants.hoodMaxPos;
    //   flywheelVelocity = 100;
    // }

    // emergency trench hood align
    if (RobotState.getInstance().nearTrench()) {
      hoodPosition = ShooterConstants.HoodConstants.hoodMinPos;
    }

    // configure parameters with calculated values
    latestParameters = new ShootingParameters(isValid, turretAngle, hoodPosition, flywheelVelocity);

    // Log calculated values
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/Parameters", latestParameters);
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/LookaheadPose", lookaheadPose);
    // Logger.recordOutput(
    //     "Subsystems/Shooter/ShotCalculator/TurretToTargetDistance",
    //     lookaheadTurretToTargetDistance);

    SmartDashboard.putBoolean(
        "Is Under Tower", RobotState.getInstance().isUnderTower(lookaheadPose));
    SmartDashboard.putBoolean("Is Behind Hub", RobotState.getInstance().isBehindHub(lookaheadPose));
    SmartDashboard.putBoolean("Is Under Trench", RobotState.getInstance().nearTrench());

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
