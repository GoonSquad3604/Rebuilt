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
import java.util.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public enum ShooterTarget {
    HUB,
    LEFT_PASS,
    RIGHT_PASS,
    FORWARD
  }

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
  }

  // MARK: - Drive & vision methods

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
    return AllianceFlipUtil.apply(pose).getY() > FieldConstants.Hub.topCenterPoint.getY()
      ? true
      : false;
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

  @AutoLogOutput
  public Optional<Rotation2d> getTurretAngle(double timestamp) {
    return turretAngleBuffer.getSample(timestamp);
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



  /** Adds a new odometry sample from the drive subsystem. */
  // public void addOdometryObservation(OdometryObservation observation) {
  //   // Update odometry pose
  //   Twist2d twist = kinematics.toTwist2d(0.0, 0.0);
  //   // lastWheelPositions = observation.wheelPositions();
  //   Pose2d lastOdometryPose = odometryPose;
  //   odometryPose = odometryPose.exp(twist);

  //   // Replace odometry pose with gyro if present
  //   observation.gyroAngle.ifPresent(
  //       gyroAngle -> {
  //         // Add offset to measured angle
  //         Rotation2d angle = gyroAngle.plus(gyroOffset);
  //         odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
  //       });

  //   // Add pose to buffer at timestamp
  //   poseBuffer.addSample(observation.timestamp(), odometryPose);

  //   // Apply odometry delta to vision pose estimate
  //   Twist2d finalTwist = lastOdometryPose.log(odometryPose);
  //   estimatedPose = estimatedPose.exp(finalTwist);
  // }

  // /** Adds a turret pose observation from the turret subsystem */
  // public void addTurretObservation(TurretObservation observation) {
  //   turretAngleBuffer.addSample(observation.timestamp(), observation.turretAngle);
  // }

  // /** Adds a new vision pose observation from the vision subsystem. */
  // public void addVisionObservation(VisionObservation observation) {
  //   // If measurement is old enough to be outside the pose buffer's timespan, skip.
  //   try {
  //     if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSec > observation.timestamp()) {
  //       return;
  //     }
  //   } catch (NoSuchElementException ex) {
  //     return;
  //   }

  //   // Get odometry based pose at timestamp
  //   var sample = poseBuffer.getSample(observation.timestamp());
  //   if (sample.isEmpty()) {
  //     // exit if not there
  //     return;
  //   }

  //   // Calculate transforms between odometry pose and vision sample pose
  //   var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
  //   var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());

  //   // Shift estimated pose backwards to sample time
  //   Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

  //   // Calculate 3 x 3 vision matrix
  //   var r = new double[3];
  //   for (int i = 0; i < 3; ++i) {
  //     r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
  //   }

  //   // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
  //   // and C = I. See wpimath/algorithms.md.
  //   Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
  //   for (int row = 0; row < 3; ++row) {
  //     double stdDev = qStdDevs.get(row, 0);
  //     if (stdDev == 0.0) {
  //       visionK.set(row, row, 0.0);
  //     } else {
  //       visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
  //     }
  //   }

  //   // Calculate the transform from the shifted estimate to the observation pose
  //   Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose().toPose2d());

  //   // Scale the transform by the Kalman gain
  //   var kTimesTransform =
  //       visionK.times(
  //           VecBuilder.fill(
  //               transform.getX(), transform.getY(), transform.getRotation().getRadians()));
  //   Transform2d scaledTransform =
  //       new Transform2d(
  //           kTimesTransform.get(0, 0),
  //           kTimesTransform.get(1, 0),
  //           Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

  //   // Recalculate the current estimate by applying the scaled transform to the old estimate
  //   // then shifting forwards using odometry data
  //   estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  // }

  // public record OdometryObservation(
  //     double timestamp, SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle) {}

  // public record VisionObservation(double timestamp, Pose3d visionPose, Matrix<N3, N1> stdDevs) {}

  // public record TurretObservation(double timestamp, Rotation2d turretAngle) {}
}
