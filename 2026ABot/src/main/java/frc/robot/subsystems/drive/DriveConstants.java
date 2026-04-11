package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {

  public static final double PATHFIND_MAX_SPEED = 2.0;
  public static final double PATHFIND_MAX_ACCEL = 3.0;
  public static final double PATHFIND_MAX_SPEED_ANGULAR = 540;
  public static final double PATHFIND_MAX_ACCEL_ANGULAR = 360;

  public static final double ANGLE_KP = 4.0;
  public static final double ANGLE_KD = 0.0;
  public static final double ANGLE_MAX_VELOCITY = 12.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;

  public static final double DRIVE_KP = 0.9;
  public static final double DRIVE_KD = 0.005;
  public static final double DRIVE_MAX_VELOCITY = 8;
  public static final double DRIVE_MAX_ACCELERATION = 20;

  public static final double CLIMB_DRIVE_MAX_VELOCITY = 2;
  public static final double CLIMB_DRIVE_MAX_ACCELERATION = 10;

  public static final Pose2d leftClimbPos =
      new Pose2d(1.638, 4.183, new Rotation2d(Units.degreesToRadians(-90)));
  public static final Pose2d rightClimbPos =
      new Pose2d(1.638, 3.319, new Rotation2d(Units.degreesToRadians(-90)));

  public static final Pose2d leftClimbFirstPose =
      new Pose2d(2.5, 4.183, new Rotation2d(Units.degreesToRadians(-90)));
  public static final Pose2d rightClimbFirstPose =
      new Pose2d(2.5, 3.319, new Rotation2d(Units.degreesToRadians(-90)));

  public static final double checkClimbX = 1.775;
  public static final double climbX = 1.638;

  public static final double climbLeftY = 4.183;
  public static final double climbRightY = 3.319;

  public static final Pose2d towerDeadzonePos = new Pose2d(0.527, 3.749, new Rotation2d());
}
