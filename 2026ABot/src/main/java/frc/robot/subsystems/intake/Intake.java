// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
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

  private SysIdRoutine hingeSysID;
  private SysIdRoutine rollerSysID;

  private double lastTimestamp = 0.0;
  private boolean kickDirectionUp = true;

  public enum IntakeWantedState {
    IDLE,
    DEPLOY,
    STOW,
    INTAKE,
    KICK,
    VOMIT
  }

  private enum IntakeCurrentState {
    IDLING,
    DEPLOYING,
    STOWING,
    INTAKING,
    KICKING,
    VOMITING,
  }

  private IntakeWantedState wantedState = IntakeWantedState.IDLE;
  private IntakeCurrentState currentState = IntakeCurrentState.IDLING;

  /** Creates a new Intake. */
  public Intake(RollerSystemIOPhoenix rollerSystemIO, HingeIOPhoenix hingeIO) {
    this.rollerSystemIO = rollerSystemIO;
    this.hingeIO = hingeIO;

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
                    Logger.recordOutput("Subsystems/Intake/Roller/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> rollerSystemIO.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerSystemIO.updateInputs(rollerSystemInputs);
    hingeIO.updateInputs(hingeInputs);
    Logger.processInputs("Subsystems/Intake/Rollers", rollerSystemInputs);
    Logger.processInputs("Subsystems/Intake/Hinge", hingeInputs);

    IntakeCurrentState newState = handleStateTransitions();
    double newTimestamp = Timer.getFPGATimestamp();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Intake", currentState);
      applyStates();
    } else {
      if (currentState == IntakeCurrentState.KICKING
          && lastTimestamp < newTimestamp - IntakeConstants.HingeConstants.kickInterval) {
        applyStates();
      }
    }
  }
  public void setWantedState(IntakeWantedState state){
    wantedState = state;
  }

  private IntakeCurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> IntakeCurrentState.IDLING;
      case DEPLOY -> IntakeCurrentState.DEPLOYING;
      case STOW -> IntakeCurrentState.STOWING;
      case INTAKE -> IntakeCurrentState.INTAKING;
      case KICK -> IntakeCurrentState.KICKING;
      case VOMIT -> IntakeCurrentState.VOMITING;
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
      case INTAKING:
        intake();
        break;
      case KICKING:
        kick();
        break;
      case VOMITING:
        vomit();
        break;
    }
  }

  private void idling() {
    rollerSystemIO.setPower(0);
    hingeIO.setPower(0);
  }

  private void deploy() {
    hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
  }

  private void stow() {
    rollerSystemIO.setPower(0);
    hingeIO.setPosition(IntakeConstants.HingeConstants.stowedPosition);
  }

  private void intake() {
    rollerSystemIO.setPower(IntakeConstants.RollerConstants.intakeSpeed);
    hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
  }

  private void kick() {
    if (MathUtil.isNear(
        IntakeConstants.HingeConstants.kickPosition,
        hingeIO.getPosition(),
        IntakeConstants.HingeConstants.nearPositionTolerance)) {
      // kick down
      hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
    } else {
      // kick up
      hingeIO.setPosition(IntakeConstants.HingeConstants.kickPosition);
    }
  }

  private void vomit() {
    rollerSystemIO.setPower(IntakeConstants.RollerConstants.vomitSpeed);
    hingeIO.setPosition(IntakeConstants.HingeConstants.deployedPosition);
  }

  // testing only:
  public void setRollerPower(double power) {
    rollerSystemIO.setPower(power);
  }

  public void setHingePower(double power) {
    hingeIO.setPower(power);
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
