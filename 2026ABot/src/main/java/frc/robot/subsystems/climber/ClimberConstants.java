package frc.robot.subsystems.climber;

/** Add your docs here. */
public final class ClimberConstants {
  public static final int outerMotorID = 4;
  public static final int innerMotorID = 15;
  // public static final int leftClimberRangeID = 30;
  // public static final int rightClimberRangeID = 31;
  public static final int centerClimberRangeID = 32;

  public static final int outerEncoderID = 28;
  public static final int innerEncoderID = 26;

  public static final double innerP = 110;
  public static final double innerI = 0;
  public static final double innerD = 0;
  public static final double innerS = 1.0767;
  public static final double innerV = 15.014;
  public static final double innerA = 0;

  public static final double outerP = 110;
  public static final double outerI = 0;
  public static final double outerD = 0;
  public static final double outerS = 1.0767;
  public static final double outerV = 15.014;
  public static final double outerA = 0;

  public static final double innerStowedPosition = 0.74;
  public static final double outerStowedPosition = 0.451;

  public static final double outerClimbL1PositionAuto = 0.60;

  // step 0 - deploy both
  public static final double innerDeployedPosition = 0.348;
  public static final double outerDeployedPosition = 0.844;

  // step 1 - climb on low rung with outer hooks
  public static final double outerClimbL1Position = 0.50;
  public static final double checkOuterClimbL1Position = 0.547;
  public static final double checkOuterClimbL1PositionRed = 0.5425; //

  // step 2 - grab mid rung with inner hooks

  public static final double innerGrabL2Position = 0.350;
  public static final double innerGrabL2PositionRed = .358; //

  // step 3 - L1 to deployed position
  public static final double checkOuterDeployedPosition = 0.8;

  // step 4 - climb on mid rung with inner hooks
  public static final double innerClimbL2Position = 0.67;
  public static final double checkInnerClimbL2Position = 0.650;
  public static final double checkInnerClimbL2PositionRed = 0.655;

  // step 5 - grab high rung with outer hooks
  public static final double outerGrabL3Position = 0.838;
  public static final double outerGrabL3PositionRed = 0.828; //

  // step 6 - release inner hooks from mid rung
  public static final double outerHopPosition = 0.841;
  public static final double innerReleaseL2Position = 0.615;

  // step 7 - climb on high rung with outer hooks (and disable inner pid)
  public static final double outerClimbL3Position = 0.500; // placeholder

  public static final double atSetpointTolerance = 0.005; // placeholder

  public static final double innerAcceleration = 8;
  public static final double outerAcceleration = 8;
  public static final double innerVelocity = 4;
  public static final double outerVelocity = 4;
}
