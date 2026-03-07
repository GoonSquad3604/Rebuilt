package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {

  public static final double PATHFIND_MAX_SPEED = 5.0;
  public static final double PATHFIND_MAX_ACCEL = 3.0;
  public static final double PATHFIND_MAX_SPEED_ANGULAR = 540;
  public static final double PATHFIND_MAX_ACCEL_ANGULAR = 360;

  public static final double ANGLE_KP = 4.0;
  public static final double ANGLE_KD = 0.0;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;

  public static final double DRIVE_KP = 0.9;
  public static final double DRIVE_KD = 0.005;
  public static final double DRIVE_MAX_VELOCITY = 8;
  public static final double DRIVE_MAX_ACCELERATION = 20;

  public static final Pose2d leftClimbPos =
      new Pose2d(1.531, 4.197, new Rotation2d(Units.degreesToRadians(-90)));
  public static final Pose2d rightClimbPos =
      new Pose2d(1.523, 3.331, new Rotation2d(Units.degreesToRadians(-90)));
}
