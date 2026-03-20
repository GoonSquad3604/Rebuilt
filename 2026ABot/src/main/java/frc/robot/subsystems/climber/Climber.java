package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIOPhoenix climberIO = new ClimberIOPhoenix();
  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private final Alert climberOuterMotorDisconnected;
  private final Alert climberInnerMotorDisconnected;
  private final Alert climberOuterEncoderDisconnected;
  private final Alert climberInnerEncoderDisconnected;

  private SysIdRoutine climberInnerSysId;
  private SysIdRoutine climberOuterSysId;

  private int manualClimbStep = 0;
  private boolean beganClimbing;
  // private boolean declimbing = false;

  private boolean beganAutoClimb = false;

  public enum ClimberWantedState {
    IDLE,
    DEPLOY,
    CLIMB,
    CLIMB_IN_AUTO,
    DECLIMB,
    STOW,
    DEPLOY_OUTER,
    CLIMB_LOW_RUNG,
    GRAB_MID_RUNG,
    CLIMB_MID_RUNG,
    GRAB_HIGH_RUNG,
    RELEASE_INNER,
    CLIMB_HIGH_RUNG,
  }

  public enum ClimberCurrentState {
    IDLING,

    STOWING,

    DEPLOYED,
    DEPLOYING,

    CLIMBING_IN_AUTO,
    DECLIMB,

    CLIMBING_LOW_RUNG,
    ON_LOW_RUNG,

    GRABBING_MID_RUNG,
    GRABBED_MID_RUNG,
    DEPLOYING_OUTER,
    DEPLOYED_OUTER,
    CLIMBING_MID_RUNG,
    ON_MID_RUNG,

    GRABBING_HIGH_RUNG,
    GRABBED_HIGH_RUNG,
    RELEASING_INNER,
    RELEASED_INNER,
    CLIMBING_HIGH_RUNG,
    ON_HIGH_RUNG
  }

  private ClimberWantedState wantedState = ClimberWantedState.IDLE;
  private ClimberCurrentState currentState = ClimberCurrentState.IDLING;

  public Climber(ClimberIOPhoenix climberIO) {
    this.climberIO = climberIO;

    climberOuterMotorDisconnected =
        new Alert("Climber Outer Motor Disconnected", Alert.AlertType.kWarning);
    climberInnerMotorDisconnected =
        new Alert("Climber Inner Motor Disconnected", Alert.AlertType.kWarning);
    climberOuterEncoderDisconnected =
        new Alert("Climber Outer Encoder Disconnected", Alert.AlertType.kWarning);
    climberInnerEncoderDisconnected =
        new Alert("Climber Inner Encoder Disconnected", Alert.AlertType.kWarning);

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
    // Logger.recordOutput("Subsystems/Climber/ManualClimbStep", manualClimbStep);

    SmartDashboard.putBoolean("away from tower?", RobotState.getInstance().isAwayFromTower());

    ClimberCurrentState newState = handleStateTransitions();
    if (newState != currentState || currentState == ClimberCurrentState.DECLIMB) {
      if (!beganClimbing) {
        currentState = newState;
        Logger.recordOutput("Subsystems/Climber/CurrentState", currentState);
        applyStates();
      }
    }

    SmartDashboard.putBoolean("climber deployed", isDeployed());

    Logger.recordOutput("Subsystems/Climber/WantedState", wantedState);
    climberOuterMotorDisconnected.set(!climberInputs.outerMotorConnected);
    climberInnerMotorDisconnected.set(!climberInputs.innerMotorConnected);
    climberOuterEncoderDisconnected.set(!climberInputs.outerEncoderConnected);
    climberInnerEncoderDisconnected.set(!climberInputs.innerEncoderConnected);
  }

  public void setWantedState(ClimberWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public ClimberCurrentState getCurrentState() {
    return currentState;
  }

  private ClimberCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> ClimberCurrentState.IDLING;
      case STOW -> !isStowed() ? ClimberCurrentState.STOWING : ClimberCurrentState.IDLING;
      case CLIMB_IN_AUTO -> ClimberCurrentState.CLIMBING_IN_AUTO;
      case DECLIMB -> ClimberCurrentState.DECLIMB;

        // not at mid rung? go to mid rung;
        // if near deploy position, update state from deploying to deployed
      case DEPLOY -> nearPosition(ClimberConstants.outerDeployedPosition, "outer")
              && nearPosition(ClimberConstants.innerDeployedPosition, "inner")
          ? ClimberCurrentState.DEPLOYED
          : ClimberCurrentState.DEPLOYING;
        // if near outers's climb low rung position, update state from climbing low to on low
      case CLIMB_LOW_RUNG -> nearPosition(ClimberConstants.checkClimbL1Position, "outer")
          ? ClimberCurrentState.ON_LOW_RUNG
          : ClimberCurrentState.CLIMBING_LOW_RUNG;

        // if near inner's grab mid rung position, update state from grabbing mid rung to grabbed
        // mid rung
      case GRAB_MID_RUNG -> nearPosition(ClimberConstants.innerGrabL2Position, "inner")
          ? ClimberCurrentState.GRABBED_MID_RUNG
          : ClimberCurrentState.GRABBING_MID_RUNG;

      case DEPLOY_OUTER -> ClimberCurrentState.DEPLOYING_OUTER;

        // if near inner's mid rung climb position, update state from climbing mid to climbed mid
      case CLIMB_MID_RUNG -> nearPosition(ClimberConstants.checkInnerClimbL2Position, "inner")
          ? ClimberCurrentState.ON_MID_RUNG
          : ClimberCurrentState.CLIMBING_MID_RUNG;

        // if near outer high rung's grab position, update state from grabbing high to high rung
      case GRAB_HIGH_RUNG -> nearPosition(ClimberConstants.checkOuterGrabL3Position, "outer")
          ? ClimberCurrentState.GRABBED_HIGH_RUNG
          : ClimberCurrentState.GRABBING_HIGH_RUNG;

        // if near inner's grab position, update state from releasing inner to released inner
      case RELEASE_INNER -> nearPosition(ClimberConstants.innerReleaseL2Position, "inner")
          ? ClimberCurrentState.RELEASED_INNER
          : ClimberCurrentState.RELEASING_INNER;

        // if near outter high rung climb position, update state from climbing high rung to climbed
        // high
      case CLIMB_HIGH_RUNG -> nearPosition(ClimberConstants.outerClimbL3Position, "outer")
          ? ClimberCurrentState.ON_HIGH_RUNG
          : ClimberCurrentState.CLIMBING_HIGH_RUNG;
      case CLIMB -> throw new UnsupportedOperationException("Unimplemented case: " + wantedState);
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
      case CLIMBING_IN_AUTO:
        climbInAuto();
        break;
      case DECLIMB:
        declimb();
        break;
      case CLIMBING_LOW_RUNG:
        climbLowRung();
        break;
      case CLIMBING_MID_RUNG:
        climbMidRung();
        break;
      case CLIMBING_HIGH_RUNG:
        climbHighRung();
        break;
      case RELEASING_INNER:
        releaseInner();
        break;
      case DEPLOYING_OUTER:
        deployOuter();
        break;
      case GRABBING_HIGH_RUNG:
        grabHighRung();
        break;
      case GRABBING_MID_RUNG:
        grabmidRung();
        break;

        // transition states:
      case DEPLOYED_OUTER:
      case GRABBED_HIGH_RUNG:
      case GRABBED_MID_RUNG:
      case ON_HIGH_RUNG:
      case ON_LOW_RUNG:
      case ON_MID_RUNG:
      case RELEASED_INNER:
      case DEPLOYED:
        break;
    }
  }

  private void idling() {
    // climberIO.setInnerPower(0);
    // climberIO.setOuterPower(0);
  }

  private void deploy() {
    climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
    climberIO.setInnerPosition(ClimberConstants.innerDeployedPosition);
  }

  private void stow() {
    climberIO.setOuterPosition(ClimberConstants.outerStowedPosition);
    climberIO.setInnerPosition(ClimberConstants.innerStowedPosition);
  }

  private void climbInAuto() {
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1PositionAuto);
  }

  private void declimb() {
    if (RobotState.getInstance().isAwayFromTower()) {
      this.wantedState = ClimberWantedState.STOW;
    } else {
      climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
    }
  }

  // public boolean isDeclimbing() {
  //   return declimbing;
  // }

  private void climbLowRung() {
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1Position);
  }

  // private void climbLowRungInAuto() {
  //   climberIO.setOuterPosition(ClimberConstants.outerClimbL1PositionAuto);
  // }

  private void climbMidRung() {
    climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
  }

  private void climbHighRung() {
    climberIO.setOuterPosition(ClimberConstants.outerClimbL3Position);
  }

  private void releaseInner() {
    climberIO.setInnerPosition(ClimberConstants.innerReleaseL2Position);
  }

  private void deployOuter() {
    climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
  }

  private void grabHighRung() {
    climberIO.setOuterPosition(ClimberConstants.outerGrabL3Position);
  }

  private void grabmidRung() {
    climberIO.setInnerPosition(ClimberConstants.innerGrabL2Position);
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

  public boolean isDeployed() {
    return MathUtil.isNear(
            ClimberConstants.innerDeployedPosition,
            climberIO.getInnerPosition(),
            ClimberConstants.atSetpointTolerance)
        && MathUtil.isNear(
            ClimberConstants.outerDeployedPosition,
            climberIO.getOuterPosition(),
            ClimberConstants.atSetpointTolerance);
  }

  public boolean isOuterDeployed() {
    return MathUtil.isNear(
        ClimberConstants.outerDeployedPosition,
        climberIO.getOuterPosition(),
        ClimberConstants.atSetpointTolerance);
  }

  public boolean nearPosition(double position, String selectedMotor) {
    return MathUtil.isNear(
        position,
        selectedMotor.equals("inner") ? climberIO.getInnerPosition() : climberIO.getOuterPosition(),
        ClimberConstants.atSetpointTolerance);
  }

  public void updateClimberState(ClimberCurrentState newState) {
    currentState = newState;
    // Logger.recordOutput("Subsystems/Climber/CurrentState", currentState);
  }

  public void resetClimbStep() {
    manualClimbStep = 0;
    beganClimbing = false;
  }

  public void progressManualClimb() {
    beganClimbing = true;
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
        // inner hooks hooked low rung, deploy outer
        climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
        break;
      case 3:
        // ready to pull up on mid rung
        climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
        break;

      case 4:
        // hook with outer rungs
        climberIO.setOuterPosition(ClimberConstants.outerGrabL3Position);
        break;
      case 5:
        // outer should have attached, release inner
        climberIO.setInnerPosition(ClimberConstants.innerReleaseL2Position);
        break;
      case 6:
        // pull up on high rung
        climberIO.setOuterPosition(ClimberConstants.outerClimbL3Position);
        climberIO.setInnerPower(0.0);
        break;
    }
    manualClimbStep++;
  }

  public void setAutoClimbing(boolean newValue) {
    beganAutoClimb = newValue;
  }

  public boolean isAutoClimbing() {
    return beganAutoClimb;
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
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1PositionAuto);
  }

  public void TESTClimbL2() {
    climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
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
