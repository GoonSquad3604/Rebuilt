package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIOPhoenix io;

  private final SysIdRoutine climber1SysId;
  private final SysIdRoutine climber2SysId;

  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private Pose2d wantedClimbPose;

  public enum ClimberWantedState {
    IDLE,
    STOWED,
    DEPLOY,
    LOWER_TO_GROUND,
    CLIMB_L1,
    CLIMB_L2,
    CLIMB_L3,
    AUTO_CLIMB
  }

  public enum CurrentState {
    IDLING,

    DEPLOYED,
    DEPLOYING,

    STOWING,
    STOWED,

    ON_L1,
    ON_L2,
    ON_L3,

    LOWERING_TO_GROUND,
    CLIMBING_TO_L1,
    CLIMBING_TO_L2,
    CLIMBING_TO_L3
  }

  private ClimberWantedState wantedState = ClimberWantedState.STOWED;
  private CurrentState currentState = CurrentState.STOWED;

  public Climber(ClimberIOPhoenix io) {
    this.io = io;

    climber1SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Climber/Climber1/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setClimber1OpenLoop(voltage.in(Volts)), null, this));

    climber2SysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Subsystems/Shooter/Kicker/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setClimber2OpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.processInputs("Subsystems/Climber", inputs);

    Logger.recordOutput("Subsystems/Climber/WantedClimberState", wantedState);
    Logger.recordOutput("Subsystems/Climber/CurrentClimberState", wantedState);

    currentState = handleStateTransitions();
    // applyStates();
  }

  public void setWantedState(ClimberWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private CurrentState handleStateTransitions() {

    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLING;
      case DEPLOY:
        yield (MathUtil.isNear(ClimberConstants.outerHookDeployedPos, io.getPositionOuter(), 1) 
          && MathUtil.isNear(ClimberConstants.innerHookDeployedPos, io.getPositionInner(), 1)) 
          ? CurrentState.DEPLOYED 
          : CurrentState.DEPLOYING;
      case STOWED:
        yield CurrentState.STOWING;
      case LOWER_TO_GROUND:
        yield CurrentState.LOWERING_TO_GROUND;
      case CLIMB_L1:
        yield CurrentState.CLIMBING_TO_L1;
      case CLIMB_L2:
        yield CurrentState.CLIMBING_TO_L2;
      case CLIMB_L3:
        yield CurrentState.CLIMBING_TO_L3;
      case AUTO_CLIMB:
        if (currentState == CurrentState.STOWED) {
          yield CurrentState.DEPLOYING;
        } else if (currentState == CurrentState.DEPLOYED
            && RobotState.getInstance().getPose() == wantedClimbPose) {
          yield CurrentState.CLIMBING_TO_L1;
        } else if (currentState == CurrentState.ON_L1) {
          yield CurrentState.CLIMBING_TO_L2;
        } else if (currentState == CurrentState.ON_L2) {
          yield CurrentState.CLIMBING_TO_L3;
        } else if (currentState == CurrentState.ON_L3) {
          yield CurrentState.ON_L3;
        } else {
          yield CurrentState.IDLING;
        }
    };
  }

  private void applyStates() {

    switch (currentState) {
      case IDLING:
        stop();
        break;
      case DEPLOYING:
        deploy();
        break;
      case STOWING:
        stow();
        break;
      case LOWERING_TO_GROUND:
        decesend();
        break;
      case CLIMBING_TO_L1:
        climbOuter();
        break;
      case CLIMBING_TO_L3:
        climbInner();
        break;
      default:
        break;
    }
  }
  
  private void stop(){
    io.setPowerOuter(0);
    io.setPowerInner(0);
  }

  private void deploy() {
    io.setPositionInner(ClimberConstants.innerHookDeployedPos);
    io.setPositionOuter(ClimberConstants.innerHookDeployedPos);
  }

  private void stow() {
    io.setPositionOuter(ClimberConstants.innerHookStowedPos);
    io.setPositionInner(ClimberConstants.outerHookStowedPos);
  }

  private void decesend() {
    io.setPositionInner(ClimberConstants.innerHookDeployedPos);
    io.setPositionOuter(ClimberConstants.innerHookDeployedPos);
  }

  private void climbOuter() {}

  private void climbInner() {}

  public void setPowerOuter(double power) {
    io.setPowerOuter(power);
  }

  public void setPowerInner(double power) {
    io.setPowerInner(power);
  }

  public void setWantedClimberPose(Pose2d wantedClimbPose) {
    this.wantedClimbPose = wantedClimbPose;
  }

  public void setHook1Power(double power) {
    io.setPowerOuter(power);
  }

  public void setHook2Power(double power) {
    io.setPowerInner(power);
  }

  public Command climber1SysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setClimber1OpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climber1SysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command climber1SysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setClimber1OpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climber1SysId.dynamic(direction));
  }

  public Command climber2SysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setClimber2OpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climber2SysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command climber2SysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setClimber2OpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(climber2SysId.dynamic(direction));
  }
}
