package frc.robot.subsystems.shooter;

public final class ShooterConstants {

  public static final double loopPeriodSecs = 0.02;

  public static final class HoodConstants {

    public static final int hoodID = 14;
    public static final int hoodEncoderID = 27;

    public static final double hoodP = 0;
    public static final double hoodI = 0;
    public static final double hoodD = 0;
    public static final double hoodS = 0;
    public static final double hoodV = 0;
  }

  public static final class LauncherConstants {

    public static final int launcherID = 9;
    // public static final int launcherEncoderID = 0;

    public static final double launcherP = 0.061342;
    public static final double launcherI = 0;
    public static final double launcherD = 0;
    public static final double launcherS = 0.14383;
    public static final double launcherV = 0.12361;
    public static final double launcherA = 0.038879;
  }

  public static final class TurretConstants {

    public static final int turretID = 11;
    public static final int turretEncoderID = 26;

    public static final double turretP = 0;
    public static final double turretI = 0;
    public static final double turretD = 0;
    public static final double turretS = 0;
    public static final double turretV = 0;
  }

  public static final class KickerConstants {
    public static final int kickerID = 6;

    public static final double kickerP = 1.6687E-07;
    public static final double kickerI = 0;
    public static final double kickerD = 0;
    public static final double kickerS = 0.38727;
    public static final double kickerV = 0.0018981;
    public static final double kickerA = 0.00017089;
  }
}
