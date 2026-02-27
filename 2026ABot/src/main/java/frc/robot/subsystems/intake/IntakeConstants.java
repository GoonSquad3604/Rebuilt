// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeConstants {

  public static final class HingeConstants {

    public static final int motorID = 0;
    public static final int encoderID = 0;

    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = 0;
    public static final double A = 0;

    public static final double deployedPosition = 0.0;
    public static final double stowedPosition = 0.0;
    public static final double kickPosition = 0.0;

    public static final double kickInterval = 1.5;
    public static final double nearPositionTolerance = 0.0;
  }

  public static final class RollerConstants {

    public static final int motorID = 0;

    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double S = 0;
    public static final double V = 0;
    public static final double A = 0;

    public static final double intakeSpeed = 0.4;
    public static final double vomitSpeed = -0.4;
  }
}
