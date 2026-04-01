package frc.robot.subsystems.climber;

/** Add your docs here. */
public final class ClimberConstants {
  public static final int outerMotorID = 4;
  public static final int innerMotorID = 15;
  public static final int leftClimberRangeID = 30;
  public static final int rightClimberRangeID = 31;
  public static final int centerClimberRangeID = 32;

  public static final int outerEncoderID = 28;
  public static final int innerEncoderID = 26;

  public static final double innerP = 100;
  public static final double innerI = 0;
  public static final double innerD = 0;
  public static final double innerS = 1;
  public static final double innerV = 1;
  public static final double innerA = 0;

  public static final double outerP = 110;
  public static final double outerI = 0;
  public static final double outerD = 0;
  public static final double outerS = 1;
  public static final double outerV = 1;
  public static final double outerA = 0;

  public static final double innerStowedPosition = 0.74;
  public static final double outerStowedPosition = 0.451;

  public static final double outerClimbL1PositionAuto = 0.60;

  // step 0
  public static final double innerDeployedPosition = 0.348;
  public static final double outerDeployedPosition = 0.844;

  // step 1
  public static final double outerClimbL1Position = 0.530;
  public static final double checkClimbL1Position = 0.547;

  // step 2
  public static final double innerGrabL2Position = 0.360;

  // step 3: L1 to deployed position

  // step 4
  public static final double innerClimbL2Position = 0.680;
  public static final double checkInnerClimbL2Position = 0.644;

  // step 5
  public static final double outerGrabL3Position = 0.829;
  public static final double checkOuterGrabL3Position = 0.836;

  // step 6
  public static final double innerReleaseL2Position = 0.569;
  public static final double checkInnerReleaseL2Position = 0.569;

  // step 7 (and disable inner pid)
  public static final double outerClimbL3Position = 0.524; // placeholder

  public static final double atSetpointTolerance = 0.0075; // placeholder

  public static final double innerAcceleration = 8;
  public static final double outerAcceleration = 8;
  public static final double innerVelocity = 4;
  public static final double outerVelocity = 4;
}
