package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {

  // public static final double loopPeriodSecs = 0.02;

  public static Transform3d robotToTurret =
      new Transform3d(
          Units.inchesToMeters(-6),
          Units.inchesToMeters(2),
          Units.inchesToMeters(18),
          Rotation3d.kZero);

  public static double robotToTurretLinear = 6.3245553203;

  public static Translation2d leftPassPosition = new Translation2d(3.5, 6.125);
  public static Translation2d rightPassPosition = new Translation2d(3.5, 2.125);

  public static final class HoodConstants {

    public static final int hoodID = 9;
    public static final int hoodEncoderID = 25;

    public static final double hoodP = 18; // 25
    public static final double hoodI = 0;
    public static final double hoodD = 0.0;
    public static final double hoodS = 0.3; // 0.1
    public static final double hoodV = 0.0; // 0.25
    public static final double hoodG = 0.0;

    public static final double acceleration = 16;
    public static final double velocity = 8;

    public static final double forwardPosition = 0.368;
    public static final double hoodMaxPos = 0.725;
    public static final double hoodMinPos = 0.05;

    // ensure it is 0.05 above 0 when at minimum
    public static final double offset = -0.377;
  }

  public static final class LauncherConstants {

    public static final int launcherID = 14;

    public static final double launcherP = 0.18937;
    public static final double launcherI = 0;
    public static final double launcherD = 0;
    public static final double launcherS = 0.0955;
    public static final double launcherV = 0.12313;
    public static final double launcherA = 0.041093;

    public static final double forwardVelocity = 50;
    public static final double cleanSpeed = 0.1;

    public static final double launcherAtSetpointTolerance = 3;
  }

  // public static final class SecondaryLauncherConstants {

  //   public static final int motorID = 9;

  //   public static final double P = 0.010244;
  //   public static final double I = 0;
  //   public static final double D = 0;
  //   public static final double S = 0.36956;
  //   public static final double V = 0.1017;
  //   public static final double A = 0.0032915;

  //   public static final double forwardVelocity = 50;
  //   public static final double cleanSpeed = 0.1;

  //   public static final double atSetpointTolerance = 3;
  // }

  public static final class TurretConstants {

    public static final int turretID = 5;
    public static final int turretEncoderID = 30;

    public static final double turretP = 32;
    public static final double turretI = 0;
    public static final double turretD = 0.25;
    public static final double turretS = 0.4;
    public static final double turretV = 0;

    public static final double maxEncoderPosition = 0.927; // .75 for 180
    public static final double minEncoderPosition = 0.06; // .25 for 180

    public static final double forwardPosition = 0;
    public static final double angleAtSetpointTolerance = 30;

    public static final double turretAcceleration = 50;
    public static final double turretVelocity = 25;
  }
}
