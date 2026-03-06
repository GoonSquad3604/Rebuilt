package frc.robot.subsystems.climber;

/** Add your docs here. */
public final class ClimberConstants {
  public static final int outerMotorID = 15;
  public static final int innerMotorID = 4;

  public static final int outerEncoderID = 28;
  public static final int innerEncoderID = 29;

  public static final double innerP = 0;
  public static final double innerI = 0;
  public static final double innerD = 0;
  public static final double innerS = 0;
  public static final double innerV = 0;
  public static final double innerA = 0;

  public static final double outerP = 0;
  public static final double outerI = 0;
  public static final double outerD = 0;
  public static final double outerS = 0;
  public static final double outerV = 0;
  public static final double outerA = 0;

  // climber setpoint logic:
  // inner&outer deploy
  // outer climb l1
  // inner climb l2
  // outer deploy (cross L3)
  // outer climb l3

  public static final double innerStowedPosition = 0;
  public static final double outerStowedPosition = 0;

  public static final double innerDeployedPosition = 0;
  public static final double outerDeployedPosition = 0;

  public static final double outerClimbL1Position = 0;

  public static final double innerClimbL2Position = 0;

  public static final double outerClimbL3Position = 0;

  public static final double atSetpointTolerance = 0;
}
