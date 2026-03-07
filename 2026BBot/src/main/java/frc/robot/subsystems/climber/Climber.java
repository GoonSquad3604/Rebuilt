package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIOPhoenix climberIO = new ClimberIOPhoenix();
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private SysIdRoutine climberInnerSysId;
  private SysIdRoutine climberOuterSysId;

  private int manualClimbStep = 0;

  public enum ClimberWantedState {
    IDLE,
    STOW,
    DEPLOY,
    CLIMB_LOW_RUNG,
    UNCLIMB_LOW_RUNG,
    CLIMB_MID_RUNG,
    CLIMB_HIGH_RUNG,
    CLIMB
  }

  public enum ClimberCurrentState {
    IDLING,

    STOWING,

    DEPLOYED,
    DEPLOYING,

    CLIMBING_OFF_LOW_RUNG,
    CLIMBING_LOW_RUNG,
    ON_LOW_RUNG,

    CLIMBING_MID_RUNG,
    ON_MID_RUNG,

    CLIMBING_HIGH_RUNG,
    ON_HIGH_RUNG
  }

  private ClimberWantedState wantedState = ClimberWantedState.IDLE;
  private ClimberCurrentState currentState = ClimberCurrentState.IDLING;

  public Climber(ClimberIOPhoenix climberIO) {
    this.climberIO = climberIO;

    climberInnerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Climber/ClimberInner/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> climberIO.setInnerOpenLoop(voltage.in(Volts)), null, this));

    climberOuterSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Climber/ClimberOuter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> climberIO.setOuterOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Subsystems/Climber", climberInputs);
    Logger.recordOutput("Subsystems/Climber/ManualClimbStep", manualClimbStep);

    // ClimberCurrentState newState = handleStateTransitions();
    // if (newState != currentState) {
    //   currentState = newState;
    //   Logger.recordOutput("Subsystems/Climber/CurrentState", currentState);
    //   applyStates();
    // }

    // Logger.recordOutput("Subsystems/Climber/WantedState", wantedState);
  }

  public void setWantedState(ClimberWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private ClimberCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> ClimberCurrentState.IDLING;
      case STOW -> isStowed() ? ClimberCurrentState.STOWING : ClimberCurrentState.IDLING;
      case DEPLOY -> nearPosition(ClimberConstants.outerDeployedPosition, true)
              && nearPosition(ClimberConstants.innerDeployedPosition, false)
          ? ClimberCurrentState.DEPLOYING
          : ClimberCurrentState.DEPLOYED;
      case CLIMB_LOW_RUNG -> ClimberCurrentState.CLIMBING_LOW_RUNG;
      case UNCLIMB_LOW_RUNG -> ClimberCurrentState.CLIMBING_OFF_LOW_RUNG;
      case CLIMB_MID_RUNG -> ClimberCurrentState.CLIMBING_MID_RUNG;
      case CLIMB_HIGH_RUNG -> ClimberCurrentState.CLIMBING_HIGH_RUNG;
      case CLIMB -> decideNextClimbState();
    };
  }

  public ClimberCurrentState decideNextClimbState() {
    return switch (currentState) {
      case IDLING -> ClimberCurrentState.IDLING;
      case DEPLOYED ->
      // if near either climb position, climb low rung, else stay in deployed
      RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)
              || RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)
          ? ClimberCurrentState.CLIMBING_LOW_RUNG
          : ClimberCurrentState.DEPLOYED;
      case ON_LOW_RUNG -> ClimberCurrentState.CLIMBING_MID_RUNG;
      case ON_MID_RUNG -> ClimberCurrentState.CLIMBING_HIGH_RUNG;

        // on high, do nothing
      case ON_HIGH_RUNG -> ClimberCurrentState.ON_HIGH_RUNG;

        // transition stages, do nothing:
      case STOWING -> ClimberCurrentState.STOWING;
      case DEPLOYING -> ClimberCurrentState.DEPLOYING;
      case CLIMBING_HIGH_RUNG -> ClimberCurrentState.CLIMBING_HIGH_RUNG;
      case CLIMBING_LOW_RUNG -> ClimberCurrentState.CLIMBING_LOW_RUNG;
      case CLIMBING_MID_RUNG -> ClimberCurrentState.CLIMBING_MID_RUNG;
      case CLIMBING_OFF_LOW_RUNG -> ClimberCurrentState.CLIMBING_OFF_LOW_RUNG;
    };
  }

  private void applyStates() {

    switch (currentState) {
      case IDLING:
        idling();
        break;
      case DEPLOYING:
        deploy();
        break;
      case STOWING:
        stow();
        break;
      case CLIMBING_OFF_LOW_RUNG:
        climbOffLowRung();
        break;
      case CLIMBING_LOW_RUNG:
        climbLowRung();
        break;
      case CLIMBING_MID_RUNG:
        climbMidRung();
        break;
      default:
        break;
    }
  }

  private void idling() {
    climberIO.setInnerPower(0);
    climberIO.setOuterPower(0);
  }

  private void deploy() {
    climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
    climberIO.setInnerPosition(ClimberConstants.innerDeployedPosition);
  }

  private void stow() {
    climberIO.setOuterPosition(ClimberConstants.outerStowedPosition);
    climberIO.setInnerPosition(ClimberConstants.innerStowedPosition);
  }

  private void climbOffLowRung() {
    climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
  }

  private void climbLowRung() {
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1Position);
  }

  private void climbMidRung() {
    climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
  }

  public boolean isStowed() {
    return MathUtil.isNear(
            ClimberConstants.innerStowedPosition,
            climberIO.getInnerPosition(),
            ClimberConstants.atSetpointTolerance)
        && MathUtil.isNear(
            ClimberConstants.outerStowedPosition,
            climberIO.getOuterPosition(),
            ClimberConstants.atSetpointTolerance);
  }

  private boolean nearPosition(double position, boolean isInnerMotor) {
    return MathUtil.isNear(
        position,
        isInnerMotor ? climberIO.getInnerPosition() : climberIO.getOuterPosition(),
        ClimberConstants.atSetpointTolerance);
  }

  public void updateClimberState(ClimberCurrentState newState) {
    currentState = newState;
    Logger.recordOutput("Subsystems/Climber/CurrentState", currentState);
  }

  public void resetClimbStep() {
    manualClimbStep = 0;
  }

  public void progressManualClimb() {
    switch (manualClimbStep) {
      case 0:
        // climbers are stowed, deploy
        climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
        climberIO.setInnerPosition(ClimberConstants.innerDeployedPosition);
        break;
      case 1:
        // climbers are deployed (assuming aligned), climb L1
        climberIO.setOuterPosition(ClimberConstants.outerClimbL1Position);
        break;
      case 2:
        // outer hooks are on L1, grab onto L2 with inners
        climberIO.setInnerPosition(ClimberConstants.innerGrabL2Position);
        break;
      case 3:
        // L2 is hooked, release L1
        climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
        break;
      case 4:
        // L1 (outer) released, pull up on L2 (inner)
        climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
        break;
      case 5:
        // L2 pulled up, latch L3
        climberIO.setOuterPosition(ClimberConstants.outerGrabL3Position);
        break;
      case 6:
        // L3 is latched, release L2
        climberIO.setInnerPosition(ClimberConstants.innerReleaseL2Position);
        break;
      case 7:
        // L2 released, CLIMB L3!!!! (and release the inner pid)
        climberIO.setOuterPosition(ClimberConstants.outerClimbL3Position);
        climberIO.setInnerPower(0.0);
        break;
      default:
        break;
    }
    manualClimbStep++;
    Logger.recordOutput("Subsystems/Climber/ManualClimbStep", manualClimbStep);
  }

  // testing only, remove later:
  public void setPowerOuterRungs(double power) {
    climberIO.setOuterPower(power);
  }

  public void setPowerInnerRungs(double power) {
    climberIO.setInnerPower(power);
  }

  public void TESTDeployClimber() {
    climberIO.setInnerPosition(ClimberConstants.innerDeployedPosition);
    climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
  }

  public void TESTStowClimber() {
    climberIO.setInnerPosition(ClimberConstants.innerStowedPosition);
    climberIO.setOuterPosition(ClimberConstants.outerStowedPosition);
  }

  public void TESTClimbL1() {
    // climberIO.setInnerPosition(ClimberConstants.innerDeployedPosition);
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1Position);
  }

  public Command climberInnerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> climberIO.setInnerOpenLoop(0))
        .withTimeout(1.0)
        .andThen(climberInnerSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command climberInnerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> climberIO.setInnerOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climberInnerSysId.dynamic(direction));
  }

  public Command climberOuterSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> climberIO.setOuterOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climberOuterSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command climberOuterSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> climberIO.setOuterOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climberOuterSysId.dynamic(direction));
  }
}
