// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

  // io declaration
  private final IntakeIOPhoenix io;
  private final SysIdRoutine intakeSysId;

  private double wantedPower;
  private double previousPower = 0;

  public enum IntakeWantedState {
    IDLE,
    INTAKE,
    VOMIT,
  }

  private enum CurrentState {
    IDLING,
    INTAKING,
    VOMITING,
  }

  private CurrentState currentState = CurrentState.IDLING;
  private IntakeWantedState wantedState = IntakeWantedState.IDLE;

  /** Creates a new Intake. */
  public Intake(IntakeIOPhoenix io) {
    this.io = io;

    intakeSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Subsystems/Intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> io.setOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(intakeInputs);
    Logger.processInputs("Subsystems/Intake", intakeInputs);

    CurrentState newState = handleStateTransitions();
    if (newState != currentState) {
      currentState = newState;
      Logger.recordOutput("Subsystems/Intake/CurrentState", currentState);
      applyStates();
    }

    Logger.recordOutput("Subsystems/Intake/WantedState", wantedState);
  }

  private CurrentState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLING;
      case INTAKE:
        yield CurrentState.INTAKING;
      case VOMIT:
        yield CurrentState.VOMITING;
    };
  }

  private void applyStates() {
    switch (currentState) {
      case IDLING:
        stopIntake();
        break;
      case INTAKING:
        runIntake();
        break;
      case VOMITING:
        vomit();
        break;
    }
  }

  public void setWantedState(IntakeWantedState wantedState) {
    this.wantedState = wantedState;
  }

  private void vomit() {
    io.setPower(-0.4);
  }

  private void runIntake() {
    io.setPower(0.6);
  }

  private void stopIntake() {
    io.setPower(0);
  }

  public void setPower(double power) {
    io.setPower(power);
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public Command intakeSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> io.setOpenLoop(0.0))
        .withTimeout(1.0)
        .andThen(intakeSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command intakeSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> io.setOpenLoop(0.0)).withTimeout(1.0).andThen(intakeSysId.dynamic(direction));
  }
}
