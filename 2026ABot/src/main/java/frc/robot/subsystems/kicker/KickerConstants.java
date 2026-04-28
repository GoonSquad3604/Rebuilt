package frc.robot.subsystems.kicker;

public class KickerConstants {
  public static final int motorID = 13;

  public static final double P = 0.13469;
  public static final double I = 0.0;
  public static final double D = 0.0;
  public static final double S = 0.78342;
  public static final double V = 0.10976;
  public static final double A = 0.01112;

  public static final double shootingVelocity = 35.6;
  public static final double passingVelocity = 60;

  public static final double ejectVelocity = 75; // 20

  public static final double shootingVelocityTolerance = 1;

  public static final double minJammedVelocity = 30;
  public static final double jamCheckTimeDuration = 2.0;
  public static final double unjamVelocity = -15;
  public static final double unjamDuration = 0.5;

  public static final double cleanSpeed = 0.1;
}
