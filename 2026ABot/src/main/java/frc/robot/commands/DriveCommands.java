package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // is slowmode?
          double multiplier = slowMode.getAsBoolean() ? 0.25 : 1;

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                  omega * drive.getMaxAngularSpeedRadPerSec() * multiplier);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command joystickDriveAtAngleHub(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // calculate desired angle
              Supplier<Rotation2d> rotationSupplier;

              Translation2d target;
              if (RobotState.getInstance().getTarget() == ShooterTarget.HUB) {
                target =
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
              } else if (RobotState.getInstance().getTarget() == ShooterTarget.LEFT_PASS) {
                target = AllianceFlipUtil.apply(new Translation2d(2.203, 6.125));
              } else {
                target = AllianceFlipUtil.apply(new Translation2d(2.203, 2.125));
              }

              rotationSupplier =
                  () -> {
                    Pose2d pose = drive.getPose();
                    double rads =
                        Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX())
                            + Math.PI;
                    return Rotation2d.fromRadians(rads);
                  };

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }
  /** Angle the drive train at the closest 45 degree to get over the bump as easy as possible */
  public static Command joystickDriveAtClosest45(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, BooleanSupplier slowMode) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // calculate desired angle
              Supplier<Rotation2d> rotationSupplier;

              Rotation2d currentRotation = drive.getPose().getRotation();

              if (currentRotation.getDegrees() > 90) {
                rotationSupplier = () -> Rotation2d.fromDegrees(135);
              } else if (currentRotation.getDegrees() > 0) {
                rotationSupplier = () -> Rotation2d.fromDegrees(45);
              } else if (currentRotation.getDegrees() > -90) {
                rotationSupplier = () -> Rotation2d.fromDegrees(-45);
              } else {
                rotationSupplier = () -> Rotation2d.fromDegrees(-135);
              }

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // is slowmode?
              double multiplier = slowMode.getAsBoolean() ? 0.25 : 1;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /** Lock the Y drive coordinate to where the trench is while keeping X free to move */
  public static Command alignToTrench(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode) {

    // Create drive PID controller
    ProfiledPIDController driveYController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.DRIVE_MAX_VELOCITY, DriveConstants.DRIVE_MAX_ACCELERATION));

    // create angle PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Get linear velocity
              double yVel =
                  driveYController.calculate(drive.getPose().getY(), getTrenchY(drive.getPose()));
              if (driveYController.atSetpoint()) {
                yVel = 0;
              }

              if (isFlipped) {
                yVel = yVel * -1;
              }

              Translation2d linearVelocity =
                  new Translation2d(
                      getLinearVelocityFromJoysticks(
                              xSupplier.getAsDouble(), ySupplier.getAsDouble())
                          .getX(),
                      yVel);

              // Logger.recordOutput("RobotState/AutoDriveCalculatedVelocityY", yVel);

              // calculate desired angle to lock to for trench
              Supplier<Rotation2d> rotationSupplier;
              Rotation2d currentRotation = drive.getPose().getRotation();

              if (currentRotation.getDegrees() > -90 && currentRotation.getDegrees() < 90) {
                rotationSupplier = () -> Rotation2d.fromDegrees(0);
              } else {
                rotationSupplier = () -> Rotation2d.fromDegrees(180);
              }

              RobotState.getInstance()
                  .setTargetPathfindPose(
                      new Pose2d(
                          drive.getPose().getX(),
                          getTrenchY(drive.getPose()),
                          rotationSupplier.get()));

              // calcualte angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Apply rotation deadband
              // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              // omega = Math.copySign(omega * omega, omega);

              // is slowmode?
              double multiplier = slowMode.getAsBoolean() ? 0.4 : 1;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega /* * drive.getMaxAngularSpeedRadPerSec()*/);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            new SequentialCommandGroup(
                Commands.runOnce(() -> angleController.reset(drive.getRotation().getRadians())),
                Commands.runOnce(() -> driveYController.reset(drive.getPose().getY()))));
  }

  public static Command alignToClimbX(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode) {

    // Create drive PID controller
    ProfiledPIDController driveXController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.CLIMB_DRIVE_MAX_VELOCITY,
                DriveConstants.CLIMB_DRIVE_MAX_ACCELERATION));

    // create angle PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {

              // Get linear velocity
              double xVel =
                  driveXController.calculate(
                      drive.getPose().getX(), AllianceFlipUtil.applyX(DriveConstants.checkClimbX));
              if (driveXController.atSetpoint()) {
                xVel = 0;
              }

              Translation2d linearVelocity =
                  new Translation2d(
                      xVel,
                      getLinearVelocityFromJoysticks(
                              xSupplier.getAsDouble(), ySupplier.getAsDouble())
                          .getY());

              // Logger.recordOutput("RobotState/AutoDriveCalculatedVelocityY", yVel);

              // calculate desired angle to lock to for trench
              Supplier<Rotation2d> rotationSupplier = () -> Rotation2d.fromDegrees(-90);

              // RobotState.getInstance()
              //     .setTargetPathfindPose(
              //         new Pose2d(
              //             drive.getPose().getX(),
              //             getTrenchY(drive.getPose()),
              //             rotationSupplier.get()));

              // calcualte angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Apply rotation deadband
              // double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              // omega = Math.copySign(omega * omega, omega);

              // is slowmode?
              double multiplier = slowMode.getAsBoolean() ? 0.4 : 1;

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * multiplier,
                      omega /* * drive.getMaxAngularSpeedRadPerSec()*/);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            new SequentialCommandGroup(
                Commands.runOnce(() -> angleController.reset(drive.getRotation().getRadians())),
                Commands.runOnce(() -> driveXController.reset(drive.getPose().getX()))));
  }

  public static Command alignToClimb(Drive drive) {

    Pose2d targetPose;
    if (RobotState.getInstance().isLeftSide()) {
      targetPose = AllianceFlipUtil.apply(DriveConstants.leftClimbFirstPose);
    } else {
      targetPose = AllianceFlipUtil.apply(DriveConstants.rightClimbFirstPose);
    }

    // Create drive PID controllers
    ProfiledPIDController driveYController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.CLIMB_DRIVE_MAX_VELOCITY,
                DriveConstants.CLIMB_DRIVE_MAX_ACCELERATION));

    ProfiledPIDController driveXController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.DRIVE_MAX_VELOCITY, DriveConstants.DRIVE_MAX_ACCELERATION));

    // create angle PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              RobotState.getInstance().setTargetPathfindPose(targetPose);

              // Get y velocity
              double yVel = driveYController.calculate(drive.getPose().getY(), targetPose.getY());
              if (driveYController.atSetpoint()) {
                yVel = 0;
              }

              // Get X velocity
              double xVel = driveXController.calculate(drive.getPose().getX(), targetPose.getX());
              if (driveXController.atSetpoint()) {
                xVel = 0;
              }

              // calcualte angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetPose.getRotation().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xVel * drive.getMaxLinearSpeedMetersPerSec(),
                      yVel * drive.getMaxLinearSpeedMetersPerSec(),
                      omega /* * drive.getMaxAngularSpeedRadPerSec()*/);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            new SequentialCommandGroup(
                Commands.runOnce(() -> angleController.reset(drive.getRotation().getRadians())),
                Commands.runOnce(() -> driveYController.reset(drive.getPose().getY())),
                Commands.runOnce(() -> driveXController.reset(drive.getPose().getX()))));
  }

  private static double getTrenchY(Pose2d robotPose) {
    if (robotPose.getY() >= FieldConstants.fieldWidth / 2.0) {
      // left trench
      return FieldConstants.LeftTrench.midPoint;
    } else {
      // right trench
      return FieldConstants.RightTrench.midPoint;
    }
  }

  public static Command alignToPose(Drive drive, Pose2d targetPose) {

    // Create drive PID controllers
    ProfiledPIDController driveYController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.CLIMB_DRIVE_MAX_VELOCITY,
                DriveConstants.CLIMB_DRIVE_MAX_ACCELERATION));

    ProfiledPIDController driveXController =
        new ProfiledPIDController(
            DriveConstants.DRIVE_KP,
            0.0,
            DriveConstants.DRIVE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.DRIVE_MAX_VELOCITY, DriveConstants.DRIVE_MAX_ACCELERATION));

    // create angle PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            DriveConstants.ANGLE_KP,
            0.0,
            DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              RobotState.getInstance().setTargetPathfindPose(targetPose);

              // Get y velocity
              double yVel = driveYController.calculate(drive.getPose().getY(), targetPose.getY());
              if (driveYController.atSetpoint()) {
                yVel = 0;
              }

              // Get X velocity
              double xVel = driveXController.calculate(drive.getPose().getX(), targetPose.getX());
              if (driveXController.atSetpoint()) {
                xVel = 0;
              }

              // calcualte angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), targetPose.getRotation().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      xVel * drive.getMaxLinearSpeedMetersPerSec() * .15,
                      yVel * drive.getMaxLinearSpeedMetersPerSec() * .15,
                      omega /* * drive.getMaxAngularSpeedRadPerSec()*/);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .beforeStarting(
            new SequentialCommandGroup(
                Commands.runOnce(() -> angleController.reset(drive.getRotation().getRadians())),
                Commands.runOnce(() -> driveYController.reset(drive.getPose().getY())),
                Commands.runOnce(() -> driveXController.reset(drive.getPose().getX()))));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
