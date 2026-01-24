package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {

  public static final double zOffset = 36;
  public static final double xOffset = 11.5;
  public static final double yOffset = 0;

  public static Transform2d robotToTurret =
      new Transform2d(
          Units.inchesToMeters(11.5),
          Units.inchesToMeters(0.0),
          // Units.inchesToMeters(36),
          Rotation2d.kZero);

  // turret
  public static final double turretP = 0.075;
  public static final double turretI = 0;
  public static final double turretD = 0;
  public static final double turretFF = 0;

  public static final double minEncoderPos = 0.0;
  public static final double minAngle = 0.0;
  public static final double maxEncoderPos = 475.0;
  public static final double maxAngle = 315.0;

  // shooter
  public static final double shooterP = .6;
  public static final double shooterI = 0;
  public static final double shooterD = 0;
  public static final double shooterS = 0;
  public static final double shooterV = 0.125;
  public static final double shooterFF = 0.0;

  public static final double loopPeriodSecs = 0.02;
}
