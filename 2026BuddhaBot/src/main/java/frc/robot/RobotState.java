package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotState {

  private enum ShotTarget {
    HUB,
    LEFT_PASSING_POINT,
    RIGHT_PASSING_POINT
  }

  private ShotTarget target = ShotTarget.HUB;
  private static final double poseBufferSizeSec = 2.0;
  boolean override = false;

  // Pose estimation fields
  @AutoLogOutput private Pose2d odometryPose = Pose2d.kZero;
  @AutoLogOutput private Pose2d estimatedPose = Pose2d.kZero;
  private Rotation2d gyroOffset = Rotation2d.kZero;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  /** Reset the pose estimate and odometry pose to the given pose. */
  public void resetPose(Pose2d pose) {
    // Gyro offset is the rotation that maps the old gyro rotation (estimated - offset) to the new
    // frame of rotation
    gyroOffset = pose.getRotation().minus(odometryPose.getRotation().minus(gyroOffset));
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
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

  public boolean isLeftSide(Pose2d pose) {
    return AllianceFlipUtil.apply(pose).getY() > FieldConstants.Hub.topCenterPoint.getY()
        ? true
        : false;
  }

  public void setTarget() {
    // checks override
    if (!override) {
      // Target is the hub
      if (FieldConstants.LeftBump.farRightCorner.getX()
          > AllianceFlipUtil.apply(getPose()).getX()) {
        target = ShotTarget.HUB;
      }
      // Target is left pass
      else if (AllianceFlipUtil.apply(getPose()).getY() > FieldConstants.Hub.topCenterPoint.getY()
          && FieldConstants.LeftBump.farRightCorner.getX()
              < AllianceFlipUtil.apply(getPose()).getX()) {
        target = ShotTarget.LEFT_PASSING_POINT;
      } else {
        target = ShotTarget.RIGHT_PASSING_POINT;
      }
    }

    Logger.recordOutput("ShotTarget", target);
  }

  public void setManualTarget(ShotTarget shotTarget) {
    target = shotTarget;
    override = true;
  }

  public void endOverride() {
    override = false;
  }
}
