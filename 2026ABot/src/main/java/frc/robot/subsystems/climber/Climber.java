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
  private final Alert leftRangeDisconnected;
  private final Alert rightRangeDisconnected;
  private final Alert centerRangeDisconnected;

  private SysIdRoutine climberInnerSysId;
  private SysIdRoutine climberOuterSysId;

  private int manualClimbStep = 0;

  private boolean beganClimbing;
  private boolean beganAutoClimbing;
  private int autoClimbStep = 0;

  public enum ClimberWantedState {
    IDLE,
    STOW,
    DEPLOY,
    DEPLOY_OUTER,
    CLIMB_IN_AUTO,
    DECLIMB,
    CLIMB,
  }

  public enum ClimberCurrentState {
    IDLING,
    STOWING,
    DEPLOYING,
    DEPLOYING_OUTER,
    CLIMBING_IN_AUTO,
    DECLIMBING,

    // climb states:
    CLIMBING_LOW_RUNG,
    GRABBING_MID_RUNG,
    // deploying outer would go here
    CLIMBING_MID_RUNG,
    GRABBING_HIGH_RUNG,
    RELEASING_INNER,
    CLIMBING_HIGH_RUNG,
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
    leftRangeDisconnected = new Alert("Left CANRange Disconnected", Alert.AlertType.kWarning);
    rightRangeDisconnected = new Alert("Right CANRange Disconnected", Alert.AlertType.kWarning);
    centerRangeDisconnected = new Alert("Center CANRange Disconnected", Alert.AlertType.kWarning);

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

    // SmartDashboard.putBoolean("climber stowed", isStowed());

    SmartDashboard.putBoolean("away from tower?", RobotState.getInstance().isAwayFromTower());

    ClimberCurrentState newState = handleStateTransitions();

    if (newState != currentState || currentState == ClimberCurrentState.DECLIMBING) {
      if (!beganClimbing) {
        currentState = newState;
        Logger.recordOutput("Subsystems/Climber/CurrentState", currentState);
        applyStates();
      }
    }

    // SmartDashboard.putBoolean("climber deployed", isDeployed());
    // SmartDashboard.putBoolean("climb can proceed", canProceed());

    // SmartDashboard.putBoolean("left Climb detected", climberIO.leftClimbDetected());
    // SmartDashboard.putBoolean("right Climb detected", climberIO.rightClimbDetected());
    // SmartDashboard.putBoolean("center Climb detected", climberIO.centerClimbDetected());
    // SmartDashboard.putBoolean("climber sensors valid", sensorsValid());

    // Logger.recordOutput("Subsystems/Climber/WantedState", wantedState);

    climberOuterMotorDisconnected.set(!climberInputs.outerMotorConnected);
    climberInnerMotorDisconnected.set(!climberInputs.innerMotorConnected);
    climberOuterEncoderDisconnected.set(!climberInputs.outerEncoderConnected);
    climberInnerEncoderDisconnected.set(!climberInputs.innerEncoderConnected);
    leftRangeDisconnected.set(!climberInputs.leftClimbRangeConnected);
    rightRangeDisconnected.set(!climberInputs.rightClimbRangeConnected);
    centerRangeDisconnected.set(!climberInputs.centerClimbRangeConnected);
  }

  public void setWantedState(ClimberWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public ClimberCurrentState getCurrentState() {
    return currentState;
  }

  private ClimberCurrentState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return ClimberCurrentState.IDLING;
      case STOW:
        return !isStowed() ? ClimberCurrentState.STOWING : ClimberCurrentState.IDLING;
      case DEPLOY:
        return ClimberCurrentState.DEPLOYING;
      case DEPLOY_OUTER:
        return ClimberCurrentState.DEPLOYING_OUTER;
      case CLIMB_IN_AUTO:
        return ClimberCurrentState.CLIMBING_IN_AUTO;
      case DECLIMB:
        return ClimberCurrentState.DECLIMBING;
      case CLIMB:
        beganAutoClimbing = true;
        switch (autoClimbStep) {
          case 0:
            // climb low rung
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.CLIMBING_LOW_RUNG;
            } else {
              return currentState;
            }
          case 1:
            // if fully climbed low, grab mid rung with inner rungs
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.GRABBING_MID_RUNG;
            } else {
              return currentState;
            }
          case 2:
            // if fully grabbed mid, deploy outer rungs
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.DEPLOYING_OUTER;
            } else {
              return currentState;
            }
          case 3:
            // if fully deploy outer rungs, climb mid rung
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.CLIMBING_MID_RUNG;
            } else {
              return currentState;
            }
          case 4:
            // if fully climbed mid rung, grab high rung with outer rungs
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.GRABBING_HIGH_RUNG;
            } else {
              return currentState;
            }
          case 5:
            // if fully grabbed high rung, release inner rungs
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.RELEASING_INNER;
            } else {
              return currentState;
            }
          case 6:
            // if fully released inner, climb high rung
            if (canProceedAutoClimb()) {
              autoClimbStep++;
              return ClimberCurrentState.CLIMBING_HIGH_RUNG;
            } else {
              return currentState;
            }
          default:
            return currentState;
        }
    }
    return null;
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        idling();
        break;
      case STOWING:
        stow();
        break;
      case DEPLOYING:
        deploy();
        break;
      case DEPLOYING_OUTER:
        deployOuter();
        break;
      case CLIMBING_IN_AUTO:
        climbInAuto();
        break;
      case DECLIMBING:
        declimb();
        break;
      case CLIMBING_LOW_RUNG:
        climbLowRung();
        break;
      case GRABBING_MID_RUNG:
        grabmidRung();
        break;
        // deploy outer again
      case CLIMBING_MID_RUNG:
        climbMidRung();
        break;
      case GRABBING_HIGH_RUNG:
        grabHighRung();
        break;
      case CLIMBING_HIGH_RUNG:
        climbHighRung();
        break;
      case RELEASING_INNER:
        releaseInner();
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

  private void climbLowRung() {
    climberIO.setOuterPosition(ClimberConstants.outerClimbL1Position);
  }

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
    if (wantedState == ClimberWantedState.DEPLOY_OUTER) {
      return MathUtil.isNear(
          ClimberConstants.outerDeployedPosition,
          climberIO.getOuterPosition(),
          ClimberConstants.atSetpointTolerance);
    }
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

  public void resetClimbStep() {
    manualClimbStep = 0;
    beganClimbing = false;
    beganAutoClimbing = false;
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
        // grab mid rung with inner hooks
        climberIO.setInnerPosition(ClimberConstants.innerGrabL2Position);
        break;
      case 3:
        // preparing to pull up on mid rung
        climberIO.setOuterPosition(ClimberConstants.outerDeployedPosition);
        break;
      case 4:
        // ready to pull up on mid rung
        climberIO.setInnerPosition(ClimberConstants.innerClimbL2Position);
        break;
      case 5:
        // hook with outer rungs
        climberIO.setOuterPosition(ClimberConstants.outerGrabL3Position);
        break;
      case 6:
        // outer should have attached, release inner
        climberIO.setInnerPosition(ClimberConstants.innerReleaseL2Position);
        break;
      case 7:
        // pull up on high rung
        climberIO.setOuterPosition(ClimberConstants.outerClimbL3Position);
        climberIO.setInnerPower(0.0);
        break;
    }
    manualClimbStep++;
  }

  public boolean canProceedAutoClimb() {
    switch (autoClimbStep) {
      case 0:
        // return is deployed
        return isDeployed();
      case 1:
        // return true if outer rungs are fully climbed on low rung
        return nearPosition(ClimberConstants.checkOuterClimbL1Position, "outer");
      case 2:
        // return true if inner rungs reached the grab position
        return nearPosition(ClimberConstants.innerGrabL2Position, "inner");
      case 3:
        // return true if outer rungs are deployed
        return nearPosition(ClimberConstants.checkOuterDeployedPosition, "outer");
      case 4:
        // return true if inner rungs fully climbed mid rung
        return nearPosition(ClimberConstants.checkInnerClimbL2Position, "inner");
      case 5:
        // return true if outer rungs fully reached the grab position
        return nearPosition(ClimberConstants.outerGrabL3Position, "outer");
      case 6:
        // return true if the inner rungs reached their extended position
        return nearPosition(ClimberConstants.innerReleaseL2Position, "inner");
    }
    return false;
  }

  // returns true if center laser is detected and not right or left
  public boolean sensorsValid() {
    return climberIO.centerClimbDetected()
        && !climberIO.leftClimbDetected()
        && !climberIO.rightClimbDetected();
  }

  public boolean beganAutoClimbing() {
    return beganAutoClimbing;
  }

  // testing only, remove later:
  public void setPowerOuterRungs(double power) {
    climberIO.setOuterPower(power);
  }

  public void setPositionOuterRungs(double position) {
    climberIO.setOuterPosition(position);
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
