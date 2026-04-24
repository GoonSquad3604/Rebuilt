package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum ShooterTarget {
    HUB,
    LEFT_PASS,
    RIGHT_PASS,
    FORWARD
  }

  private Pose2d targetPose = new Pose2d();

  private ShooterTarget target = ShooterTarget.HUB;
  private ShooterTarget manualTarget = ShooterTarget.HUB;

  private boolean override = false;

  // Pose estimation fields
  private Pose2d odometryPose = Pose2d.kZero;
  private Pose2d estimatedPose = Pose2d.kZero;

  // Odometry fields
  private Rotation2d gyroOffset = Rotation2d.kZero;

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {}

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
  }

  // public boolean isLeftSide() {
  //   if (getPose().getY() >= FieldConstants.fieldWidth / 2.0) {
  //     // left
  //     return DriverStation.getAlliance().get() == Alliance.Blue ? true : false;
  //   } else {
  //     // right
  //     return DriverStation.getAlliance().get() == Alliance.Blue ? false : true;
  //   }
  // }

  // public boolean isInMiddle() {
  //   return getPose().getY() > 3.25 && getPose().getY() < 4.5;
  // }

  public boolean isUnderTower(Pose2d pose) {
    return pose.getX() < AllianceFlipUtil.apply(FieldConstants.Tower.leftUpright).getX()
        && pose.getY() > AllianceFlipUtil.apply(FieldConstants.Tower.rightUpright).getY()
        && pose.getY() < AllianceFlipUtil.apply(FieldConstants.Tower.leftUpright).getY();
  }

  public boolean isBehindHub(Pose2d pose) {
    return pose.getX() > AllianceFlipUtil.apply(FieldConstants.Hub.farLeftCorner).getX()
        && pose.getX() < AllianceFlipUtil.applyX(7.0)
        && pose.getY() > AllianceFlipUtil.apply(FieldConstants.Hub.farRightCorner).getY()
        && pose.getY() < AllianceFlipUtil.apply(FieldConstants.Hub.farLeftCorner).getY();
  }

  public boolean nearTrench() {
    // blue
    return (getPose().getX() > 3.8 && getPose().getX() < 5.8)
        // red
        || (getPose().getX() > 10.884 && getPose().getX() < 13);
  }

  /** Get the rotation of the estimated pose. */
  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void setRobotVelocity(ChassisSpeeds chassisSpeeds) {
    robotVelocity = chassisSpeeds;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  public Pose2d getPose() {
    return estimatedPose;
  }

  public ShooterTarget getTarget() {
    // checks override
    if (!override) {
      // Target is the hub
      if (FieldConstants.LeftBump.farRightCorner.getX()
          > AllianceFlipUtil.apply(getPose()).getX()) {

        target = ShooterTarget.HUB;

      } else if (AllianceFlipUtil.apply(getPose()).getY() > FieldConstants.Hub.topCenterPoint.getY()
          && FieldConstants.LeftBump.farRightCorner.getX()
              < AllianceFlipUtil.apply(getPose()).getX()) {

        // Target is left pass
        target = ShooterTarget.LEFT_PASS;

      } else {
        // Target is right pass
        target = ShooterTarget.RIGHT_PASS;
      }
    } else {
      target = manualTarget;
    }

    // Logger.recordOutput("RobotState/ShotTarget", target);
    // Logger.recordOutput("RobotState/ManualTarget", manualTarget);
    // Logger.recordOutput("RobotState/Override", override);

    return target;
  }

  public Command setManualTarget(ShooterTarget newTarget) {
    return Commands.runOnce(() -> manualTarget = newTarget);
  }

  public ShooterTarget getManualTarget() {
    return manualTarget;
  }

  public Command toggleManualShooting() {
    return Commands.runOnce(() -> override = !override);
  }

  public boolean isOverride() {
    return override;
  }

  public void setTargetPathfindPose(Pose2d newPose) {
    targetPose = newPose;
    Logger.recordOutput("RobotState/TargetPathfindPose", targetPose);
  }

  public Pose2d getTargetPathfindPose() {
    return targetPose;
  }

  public boolean atDrivePosition(Pose2d position) {
    return MathUtil.isNear(position.getX(), getPose().getX(), .1)
        && MathUtil.isNear(position.getY(), getPose().getY(), .1);
  }

  public boolean isAwayFromTower() {
    Translation2d towerPose = AllianceFlipUtil.apply(new Translation2d(1.8, 3.7));
    return towerPose.getDistance(getPose().getTranslation()) > 1.5;
  }

  public double getDistanceToHubInches() {
    Translation2d hubPose =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition =
        getPose().transformBy(GeomUtil.toTransform2d(ShooterConstants.robotToTurret));
    return Units.metersToInches(hubPose.getDistance(turretPosition.getTranslation()));
  }

  public double getDistanceToHubMeters() {
    Translation2d hubPose =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d turretPosition =
        AllianceFlipUtil.apply(getPose())
            .transformBy(GeomUtil.toTransform2d(ShooterConstants.robotToTurret));
    return hubPose.getDistance(turretPosition.getTranslation());
  }
}
