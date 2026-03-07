package frc.robot.subsystems.climber;

/** Add your docs here. */
public final class ClimberConstants {
  public static final int outerMotorID = 4;
  public static final int innerMotorID = 15;

  public static final int outerEncoderID = 29;
  public static final int innerEncoderID = 28;

  public static final double innerP = 200;
  public static final double innerI = 0;
  public static final double innerD = 0;
  public static final double innerS = 0.9;
  public static final double innerV = 1;
  public static final double innerA = 0;

  public static final double outerP = 200;
  public static final double outerI = 0;
  public static final double outerD = 0;
  public static final double outerS = 0.9;
  public static final double outerV = 1;
  public static final double outerA = 0;

  public static final double innerStowedPosition = -0.325; // bbot only
  public static final double outerStowedPosition = -0.480; // bbot only

  public static final double outerClimbL1PositionAuto = -0.425;

  // step 0
  public static final double innerDeployedPosition = -0.690;
  public static final double outerDeployedPosition = -0.132;

  // step 1
  public static final double outerClimbL1Position = -0.470; // real: -0.450;

  // step 2
  public static final double innerGrabL2Position = -0.651;

  // step 3: L1 to deployed position

  // step 4
  public static final double innerClimbL2Position = -0.355;

  // step 5
  public static final double outerGrabL3Position = -0.165;

  // step 6
  public static final double innerReleaseL2Position = -0.4;

  // step 7 (and disable inner pid)
  public static final double outerClimbL3Position = -0.46; // placeholder

  public static final double atSetpointTolerance = 0.001; // placeholder

  public static final double innerAcceleration = 8;
  public static final double outerAcceleration = 8;
  public static final double innerVelocity = 4;
  public static final double outerVelocity = 4;
}
