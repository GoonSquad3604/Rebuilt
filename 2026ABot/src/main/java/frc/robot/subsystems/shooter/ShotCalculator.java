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

  private static double passOffset = -13.0;

  public static ShotCalculator getInstance() {
    if (instance == null) instance = new ShotCalculator();
    return instance;
  }

  public record ShootingParameters(
      boolean validShootingLocation,
      double turretAngle,
      double hoodPosition,
      double flywheelVelocity,
      double passFlywheelVelocity) {}

  // Cache parameters
  private static ShootingParameters latestParameters = null;
  private static double minDistance;
  private static double maxDistance;
  private static double phaseDelay;
  private static double rotationalPhaseDelay;

  private static final InterpolatingDoubleTreeMap shotFlywheelVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passFlywheelVelocityMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap shotHoodPositionMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  static {
    minDistance = 0;
    maxDistance = 3604;
    phaseDelay = 0.1;
    rotationalPhaseDelay = 0.05;

    shotFlywheelVelocityMap.put(1.6, 46.0);
    shotFlywheelVelocityMap.put(1.8, 48.0);
    shotFlywheelVelocityMap.put(2.0, 49.5);
    shotFlywheelVelocityMap.put(2.2, 50.5);
    shotFlywheelVelocityMap.put(2.5, 52.0);
    shotFlywheelVelocityMap.put(2.75, 53.0);
    shotFlywheelVelocityMap.put(3.0, 54.0);
    shotFlywheelVelocityMap.put(3.25, 54.25);
    shotFlywheelVelocityMap.put(3.5, 54.75);
    shotFlywheelVelocityMap.put(3.75, 55.25);
    shotFlywheelVelocityMap.put(4.0, 55.75);
    shotFlywheelVelocityMap.put(4.3, 59.0);
    shotFlywheelVelocityMap.put(4.5, 61.75);
    shotFlywheelVelocityMap.put(4.75, 63.0);
    shotFlywheelVelocityMap.put(5.0, 65.0);
    shotFlywheelVelocityMap.put(5.2, 67.0);
    shotFlywheelVelocityMap.put(7.0, 80.0);

    passFlywheelVelocityMap.put(1.6, 46.0 + passOffset);
    passFlywheelVelocityMap.put(1.8, 48.0 + passOffset);
    passFlywheelVelocityMap.put(2.0, 49.5 + passOffset);
    passFlywheelVelocityMap.put(2.2, 50.5 + passOffset);
    passFlywheelVelocityMap.put(2.5, 52.0 + passOffset);
    passFlywheelVelocityMap.put(2.75, 53.0 + passOffset);
    passFlywheelVelocityMap.put(3.0, 54.0 + passOffset);
    passFlywheelVelocityMap.put(3.25, 54.25 + passOffset);
    passFlywheelVelocityMap.put(3.5, 54.75 + passOffset);
    passFlywheelVelocityMap.put(3.75, 55.25 + passOffset);
    passFlywheelVelocityMap.put(4.0, 55.75 + passOffset);
    passFlywheelVelocityMap.put(4.3, 59.0 + passOffset);
    passFlywheelVelocityMap.put(4.5, 61.75 + passOffset);
    passFlywheelVelocityMap.put(4.75, 63.0 + passOffset);
    passFlywheelVelocityMap.put(5.0, 65.0 + passOffset);
    passFlywheelVelocityMap.put(5.2, 67.0 + passOffset);
    passFlywheelVelocityMap.put(7.0, 90.0 + passOffset);

    // shotFlywheelVelocityMap.put(1.751, 44.0);
    // shotFlywheelVelocityMap.put(2.127, 45.5);
    // shotFlywheelVelocityMap.put(2.5, 46.5);
    // shotFlywheelVelocityMap.put(2.813, 47.0);
    // shotFlywheelVelocityMap.put(3.023, 50.0);
    // shotFlywheelVelocityMap.put(3.463, 52.0);
    // shotFlywheelVelocityMap.put(3.6, 56.0);
    // shotFlywheelVelocityMap.put(3.8, 60.0);
    // shotFlywheelVelocityMap.put(4.336, 61.5);
    // shotFlywheelVelocityMap.put(4.743, 63.0);
    // shotFlywheelVelocityMap.put(5.0, 67.0);
    // shotFlywheelVelocityMap.put(6.743, 81.0);

    // passFlywheelVelocityMap.put(1.751, 44.0 + passOffset);
    // passFlywheelVelocityMap.put(2.127, 45.5 + passOffset);
    // passFlywheelVelocityMap.put(2.5, 46.5 + passOffset);
    // passFlywheelVelocityMap.put(2.813, 47.0 + passOffset);
    // passFlywheelVelocityMap.put(3.023, 50.0 + passOffset);
    // passFlywheelVelocityMap.put(3.463, 52.0 + passOffset);
    // passFlywheelVelocityMap.put(3.6, 56.0 + passOffset);
    // passFlywheelVelocityMap.put(3.8, 60.0 + passOffset);
    // passFlywheelVelocityMap.put(4.336, 60.0 + passOffset);
    // passFlywheelVelocityMap.put(4.743, 63.0 + passOffset);
    // passFlywheelVelocityMap.put(5.0, 67.0 + passOffset);
    // passFlywheelVelocityMap.put(6.743, 81.0 + passOffset);

    shotHoodPositionMap.put(1.6, 0.1);
    shotHoodPositionMap.put(1.8, 0.15);
    shotHoodPositionMap.put(2.0, 0.2);
    shotHoodPositionMap.put(2.2, 0.25);
    shotHoodPositionMap.put(2.5, 0.3);
    shotHoodPositionMap.put(2.75, 0.35);
    shotHoodPositionMap.put(3.0, 0.4);
    shotHoodPositionMap.put(3.25, 0.45);
    shotHoodPositionMap.put(3.5, 0.5);
    shotHoodPositionMap.put(3.75, 0.55);
    shotHoodPositionMap.put(4.0, 0.6);
    shotHoodPositionMap.put(4.3, 0.6);
    shotHoodPositionMap.put(4.5, 0.6);
    shotHoodPositionMap.put(4.75, 0.6);
    shotHoodPositionMap.put(5.0, 0.6);
    shotHoodPositionMap.put(5.2, 0.6);

    // shotHoodPositionMap.put(1.751, 0.1794);
    // shotHoodPositionMap.put(2.127, 0.38);
    // shotHoodPositionMap.put(2.5, 0.4261);
    // shotHoodPositionMap.put(2.813, 0.4261);
    // shotHoodPositionMap.put(3.023, 0.4486);
    // shotHoodPositionMap.put(3.463, 0.471);
    // shotHoodPositionMap.put(3.6, 0.5024);
    // shotHoodPositionMap.put(3.8, 0.5203);
    // shotHoodPositionMap.put(4.336, 0.5293);
    // shotHoodPositionMap.put(4.743, 0.5383);
    // shotHoodPositionMap.put(6.743, 0.628);

    timeOfFlightMap.put(1.6, 0.92);
    timeOfFlightMap.put(1.8, 0.98);
    timeOfFlightMap.put(2.0, 1.01);
    timeOfFlightMap.put(2.2, 1.02);
    timeOfFlightMap.put(2.5, 1.045);
    timeOfFlightMap.put(2.5, 1.1);
    timeOfFlightMap.put(3.0, 1.06); // idk man
    timeOfFlightMap.put(3.25, 1.0); // idk man
    timeOfFlightMap.put(3.5, 1.0); // idk man
    timeOfFlightMap.put(3.75, 0.88); // idk man
    timeOfFlightMap.put(4.0, 0.95); // idk man
    timeOfFlightMap.put(4.3, 1.03); // idk man
    timeOfFlightMap.put(4.5, 1.03); // idk man
    timeOfFlightMap.put(4.75, 1.15); // idk man
    timeOfFlightMap.put(5.0, 1.14); // idk man
    timeOfFlightMap.put(5.2, 1.17); // idk man

    // timeOfFlightMap.put(1.751, 1.0);
    // timeOfFlightMap.put(2.127, 1.0);
    // timeOfFlightMap.put(2.5, 1.1);
    // timeOfFlightMap.put(2.813, 1.15);
    // timeOfFlightMap.put(3.023, 1.17);
    // timeOfFlightMap.put(3.463, 1.25);
    // timeOfFlightMap.put(3.8, 1.26);
    // timeOfFlightMap.put(4.336, 1.28);
    // timeOfFlightMap.put(4.743, 1.3);
    // timeOfFlightMap.put(6.743, 1.45);
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

      Rotation2d offsetRotation =
          targetPose
              .minus(lookaheadPose.getTranslation())
              .getAngle()
              .minus(
                  Rotation2d.fromRadians(
                      ShooterConstants.robotToTurretLinear
                          * robotVelocity.omegaRadiansPerSecond
                          * rotationalPhaseDelay));

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

    if (RobotState.getInstance().getTarget() != ShooterTarget.HUB) {
      hoodPosition = ShooterConstants.HoodConstants.hoodMaxPos;
    }

    // emergency trench hood align
    if (RobotState.getInstance().nearTrench()) {
      hoodPosition = ShooterConstants.HoodConstants.hoodMinPos;
    }

    // configure parameters with calculated values
    latestParameters =
        new ShootingParameters(
            isValid,
            turretAngle,
            hoodPosition,
            shotFlywheelVelocityMap.get(lookaheadTurretToTargetDistance),
            passFlywheelVelocityMap.get(lookaheadTurretToTargetDistance));

    // Log calculated values
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/Parameters", latestParameters);
    // Logger.recordOutput("Subsystems/Shooter/ShotCalculator/LookaheadPose", lookaheadPose);
    // Logger.recordOutput(
    //     "Subsystems/Shooter/ShotCalculator/TurretToTargetDistance",
    //     lookaheadTurretToTargetDistance);

    // SmartDashboard.putBoolean(
    //     "Is Under Tower", RobotState.getInstance().isUnderTower(lookaheadPose));
    SmartDashboard.putBoolean("Is Behind Hub", RobotState.getInstance().isBehindHub(lookaheadPose));
    SmartDashboard.putBoolean("Is Under Trench", RobotState.getInstance().nearTrench());

    return latestParameters;
  }

  public void clearShootingParameters() {
    latestParameters = null;
  }
}
