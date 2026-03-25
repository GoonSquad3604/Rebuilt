package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.intake.hinge.HingeIOInputsAutoLogged;
import frc.robot.subsystems.intake.hinge.HingeIOPhoenix;
import frc.robot.subsystems.intake.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.intake.rollers.RollerSystemIOPhoenix;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final RollerSystemIOPhoenix rollerSystemIO;
  private final HingeIOPhoenix hingeIO;

  private final RollerSystemIOInputsAutoLogged rollerSystemInputs =
      new RollerSystemIOInputsAutoLogged();
  private final HingeIOInputsAutoLogged hingeInputs = new HingeIOInputsAutoLogged();

  private final Alert hingeMotorDisconnected;
  private final Alert hingeEncoderDisconnected;
  private final Alert rollerSystemMotorDisconnected;

  private SysIdRoutine hingeSysID;
  private SysIdRoutine rollerSysID;

  private double lastTimestamp = 0.0;

  public enum IntakeWantedState {
    IDLE,
    INTAKE,
    // STOP_WHEELS,
    STOW,
    KICK,
    VOMIT
  }

  private enum IntakeCurrentState {
    IDLING,
    DEPLOYING,
    STOWING,
    STOWED,
    INTAKING_DEPLOYED,
    IDLE_DEPLOYED,
    KICKING,
    VOMITING,
  }

  private IntakeWantedState wantedState = IntakeWantedState.IDLE;
  private IntakeCurrentState currentState = IntakeCurrentState.IDLING;

  /** Creates a new Intake. */
  public Intake(RollerSystemIOPhoenix rollerSystemIO, HingeIOPhoenix hingeIO) {
    this.rollerSystemIO = rollerSystemIO;
    this.hingeIO = hingeIO;

    hingeMotorDisconnected = new Alert("Hinge Motor Disconnected", Alert.AlertType.kWarning);
    hingeEncoderDisconnected = new Alert("Hinge Encoder Disconnected", Alert.AlertType.kWarning);
    rollerSystemMotorDisconnected =
        new Alert("Roller System Motor Disconnected", Alert.AlertType.kWarning);

    hingeSysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("Subsystems/Intake/Hinge/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> hingeIO.setOpenLoop(voltage.in(Volts)), null, this));

    rollerSysID =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput(
                        "Subsystems/Intake/RollerSystem/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> rollerSystemIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerSystemIO.updateInputs(rollerSystemInputs);
    hingeIO.updateInputs(hingeInputs);
    Logger.processInputs("Subsystems/Intake/RollerSystem", rollerSystemInputs);
    Logger.processInputs("Subsystems/Intake/Hinge", hingeInputs);
    // Logger.recordOutput("Subsystems/Intake/Hinge/isDeployed", isDeployed());
    // Logger.recordOutput("Subsystems/Intake/Hinge/isStowed", isStowed());

    IntakeCurrentState newState = handleStateTransitions();
    double newTimestamp = Timer.getFPGATimestamp();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Intake/CurrentState", currentState);
      applyStates();
    } else {
      if (currentState == IntakeCurrentState.KICKING
          && lastTimestamp < newTimestamp - IntakeConstants.HingeConstants.kickInterval) {
        applyStates();
      }
    }
    // Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);

    hingeMotorDisconnected.set(!hingeInputs.motorConnected);
    hingeEncoderDisconnected.set(!hingeInputs.encoderConnected);
    rollerSystemMotorDisconnected.set(!rollerSystemInputs.motorConnected);
  }

  public void setWantedState(IntakeWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private IntakeCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> isDeployed() ? IntakeCurrentState.IDLE_DEPLOYED : IntakeCurrentState.IDLING;
      case STOW -> isStowed() ? IntakeCurrentState.STOWED : IntakeCurrentState.STOWING;
      case INTAKE -> !isDeployed()
          ? IntakeCurrentState.DEPLOYING
          : IntakeCurrentState.INTAKING_DEPLOYED;
      case KICK -> IntakeCurrentState.KICKING;
      case VOMIT -> IntakeCurrentState.VOMITING;
        // case STOP_WHEELS -> IntakeCurrentState.IDLE_WHEELS_HOLD_POSITION;
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
      case STOWED:
        break;
      case INTAKING_DEPLOYED:
        runRollers();
        break;
      case IDLE_DEPLOYED:
        idleDeployed();
        break;
      case KICKING:
        kick();
        break;
      case VOMITING:
        vomitRollers();
        break;
    }
  }

  private void idling() {
    rollerSystemIO.setPower(0);
    hingeIO.setPower(0);
  }

  private void idleDeployed() {
    rollerSystemIO.setPower(0);
    hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
  }

  private void deploy() {
    rollerSystemIO.setPower(IntakeConstants.RollerConstants.intakeSpeed);
    hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
  }

  private void stow() {
    rollerSystemIO.setPower(0);
    hingeIO.setPosition(IntakeConstants.HingeConstants.stowedPosition);
  }

  private void runRollers() {
    rollerSystemIO.setPower(IntakeConstants.RollerConstants.intakeSpeed);
  }

  private void kick() {
    // rollerSystemIO.setPower(IntakeConstants.RollerConstants.kickIntakeSpeed);
    rollerSystemIO.setPower(0);
    if (MathUtil.isNear(
        IntakeConstants.HingeConstants.kickPosition,
        hingeIO.getPosition(),
        IntakeConstants.HingeConstants.nearPositionTolerance)) {
      // kick down
      hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
    } else if (MathUtil.isNear(
        IntakeConstants.HingeConstants.deployedPosition,
        hingeIO.getPosition(),
        IntakeConstants.HingeConstants.nearPositionTolerance)) {
      // kick up
      hingeIO.setPosition(IntakeConstants.HingeConstants.kickPosition);
    } else {
      hingeIO.setPosition(IntakeConstants.HingeConstants.kickPosition);
    }
  }

  private void vomitRollers() {
    rollerSystemIO.setPower(IntakeConstants.RollerConstants.vomitSpeed);
  }

  public boolean isDeployed() {
    return MathUtil.isNear(
        IntakeConstants.HingeConstants.deployedPosition,
        hingeIO.getPosition(),
        IntakeConstants.HingeConstants.nearPositionTolerance);
  }

  public boolean isStowed() {
    return MathUtil.isNear(
        IntakeConstants.HingeConstants.stowedPosition,
        hingeIO.getPosition(),
        IntakeConstants.HingeConstants.nearPositionTolerance);
  }

  // testing only:
  public void setRollerPower(double power) {
    rollerSystemIO.setPower(power);
  }

  public void setHingePower(double power) {
    hingeIO.setPower(power);
  }

  public void setHingePosition(double position) {
    hingeIO.setPosition(position);
  }

  public Command hingeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> hingeIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(hingeSysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command hingeSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> hingeIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(hingeSysID.dynamic(direction));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> rollerSystemIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(rollerSysID.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> rollerSystemIO.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(rollerSysID.dynamic(direction));
  }
}
