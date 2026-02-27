package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

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

  private double wantedClimb1Power;
  private double previousClimb1Power = 0;
  private double wantedClimb2Power;
  private double previousClimb2Power = 0;

  private double wantedClimb1Pos;
  private double previousClimb1Pos = 0;
  private double wantedClimb2Pos;
  private double previousClimb2Pos = 0;

  public enum ClimberWantedState {
    IDLE,
    STOWED,
    UNCLIMB_L1,
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

    ON_LOW_RUNG,
    ON_MID_RUNG,
    ON_HIGH_RUNG,

    LOWERING_TO_GROUND,
    CLIMBING_TO_LOW_RUNG,
    CLIMBING_TO_MID_RUNG,
    CLIMBING_TO_HIGH_RUNG
  }

  private ClimberWantedState wantedState = ClimberWantedState.STOWED;
  private CurrentState currentState = CurrentState.STOWED;

  public Climber(ClimberIOPhoenix io) {
    this.io = io;
    updatePreviousPositions();

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
      case STOWED:
        yield CurrentState.STOWING;
      case UNCLIMB_L1:
        yield CurrentState.LOWERING_TO_GROUND;
      case CLIMB_L1:
        yield CurrentState.CLIMBING_TO_LOW_RUNG;
      case CLIMB_L2:
        yield CurrentState.CLIMBING_TO_MID_RUNG;
      case CLIMB_L3:
        yield CurrentState.CLIMBING_TO_HIGH_RUNG;
      case AUTO_CLIMB:
        if (currentState == CurrentState.STOWED) {
          yield CurrentState.DEPLOYING;
        } else if (currentState == CurrentState.DEPLOYED
            && RobotState.getInstance().getPose() == wantedClimbPose) {
          yield CurrentState.CLIMBING_TO_LOW_RUNG;
        } else if (currentState == CurrentState.ON_LOW_RUNG) {
          yield CurrentState.CLIMBING_TO_MID_RUNG;
        } else if (currentState == CurrentState.ON_MID_RUNG) {
          yield CurrentState.CLIMBING_TO_HIGH_RUNG;
        } else if (currentState == CurrentState.ON_HIGH_RUNG) {
          yield CurrentState.ON_HIGH_RUNG;
        } else {
          yield CurrentState.IDLING;
        }
    };
  }

  private void applyStates() {

    switch (currentState) {
      case IDLING:
        stopped();
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
      case CLIMBING_TO_LOW_RUNG:
        climbLowRung();
        break;
      case CLIMBING_TO_MID_RUNG:
        climbMidRung();
        break;
      default:
        break;
    }
  }

  private void stopped() {
    wantedClimb1Power = 0;
    wantedClimb2Power = 0;
    if (wantedClimb1Power != previousClimb1Power) {
      io.setPowerLowRung(wantedClimb1Power);
      previousClimb1Power = wantedClimb1Power;
    }

    if (wantedClimb2Power != previousClimb2Power) {
      io.setPowerMidRung(wantedClimb2Power);
      previousClimb2Power = wantedClimb2Power;
    }
  }

  private void deploy() {}

  private void stow() {}

  private void decesend() {}

  private void climbLowRung() {}

  private void climbMidRung() {}

  public void setPowerLowRung(double power) {
    io.setPowerLowRung(power);
  }

  public void setPowerMidRung(double power) {
    io.setPowerMidRung(power);
  }

  public void setWantedClimberPose(Pose2d wantedClimbPose) {
    this.wantedClimbPose = wantedClimbPose;
  }

  public void setHook1Power(double power) {
    io.setPowerLowRung(power);
  }

  public void setHook2Power(double power) {
    io.setPowerMidRung(power);
  }

  private void updatePreviousPositions() {
    previousClimb1Pos = io.getPositionLowRung();
    previousClimb2Pos = io.getPositionMidRung();
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
