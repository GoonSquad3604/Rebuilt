package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum ShooterTarget {
    HUB,
    LEFT_PASS,
    RIGHT_PASS,
    FORWARD
  }

  private Pose2d targetPose = new Pose2d();
  private Rotation2d turretAngle;

  private ShooterTarget target = ShooterTarget.HUB;
  private ShooterTarget manualTarget = ShooterTarget.HUB;

  private boolean override = false;

  private static final double poseBufferSizeSec = 2.0;
  private static final double turretAngleBufferSizeSec = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  // Pose estimation fields
  private Pose2d odometryPose = Pose2d.kZero;
  private Pose2d estimatedPose = Pose2d.kZero;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSec);
  private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
      TimeInterpolatableBuffer.createBuffer(turretAngleBufferSizeSec);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  // Odometry fields
  private Rotation2d gyroOffset = Rotation2d.kZero;

  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    Logger.recordOutput("RobotState/TargetPathfindPose", targetPose);
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

  public boolean isLeftSide(Pose2d pose) {
    if (AllianceFlipUtil.shouldFlip()) {
      return pose.getY() < FieldConstants.Hub.topCenterPoint.getY();
    } else {
      return pose.getY() > FieldConstants.Hub.topCenterPoint.getY();
    }
  }

  public boolean isInMiddle(Pose2d pose) {
    if (AllianceFlipUtil.shouldFlip()) {
      return pose.getY() < FieldConstants.Hub.leftFace.getY()
          && pose.getY() > FieldConstants.Hub.rightFace.getY();
    } else {
      return pose.getY() > FieldConstants.Hub.leftFace.getY()
          && pose.getY() < FieldConstants.Hub.rightFace.getY();
    }
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

  // @AutoLogOutput
  // public Optional<Rotation2d> getTurretAngle(double timestamp) {
  //   return turretAngleBuffer.getSample(timestamp);
  // }

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

    Logger.recordOutput("RobotState/ShotTarget", target);
    Logger.recordOutput("RobotState/ManualTarget", manualTarget);
    Logger.recordOutput("RobotState/Override", override);

    return target;
  }

  public Command setManualTarget(ShooterTarget newTarget) {
    return Commands.runOnce(() -> manualTarget = newTarget);
  }

  public Command toggleManualShooting() {
    return Commands.runOnce(() -> override = !override);
  }

  public boolean isOverride() {
    return override;
  }

  public void setTurretAngle(Rotation2d newAngle) {
    turretAngle = newAngle;
  }

  public Rotation2d getTurretAngle() {
    return turretAngle;
  }

  public void setTargetPathfindPose(Pose2d newPose) {
    targetPose = newPose;
    Logger.recordOutput("RobotState/TargetPathfindPose", targetPose);
  }

  public Pose2d getTargetPathfindPose() {
    return targetPose;
  }
}
