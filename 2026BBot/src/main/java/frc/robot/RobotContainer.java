// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOPhoenix;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIORev;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.hood.HoodIOPhoenix;
import frc.robot.subsystems.shooter.kicker.KickerIORev;
import frc.robot.subsystems.shooter.launcher.LauncherIOPhoenix;
import frc.robot.subsystems.shooter.turret.TurretIOPhoenix;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Climber climber;
  private final Indexer indexer;
  private final Intake intake;
  private final Shooter shooter;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController testController = new CommandXboxController(2);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  //   private final CommandJoystick pitBox = new CommandJoystick(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));
        climber = new Climber(new ClimberIOPhoenix());
        indexer = new Indexer(new IndexerIORev());
        intake = new Intake(new IntakeIOPhoenix());
        shooter =
            new Shooter(
                new HoodIOPhoenix(),
                new LauncherIOPhoenix(),
                new TurretIOPhoenix(),
                new KickerIORev());
        superstructure = new Superstructure(drive, intake, indexer, shooter);

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));
        climber = new Climber(new ClimberIOPhoenix());
        indexer = new Indexer(new IndexerIORev());
        intake = new Intake(new IntakeIOPhoenix());
        shooter =
            new Shooter(
                new HoodIOPhoenix(),
                new LauncherIOPhoenix(),
                new TurretIOPhoenix(),
                new KickerIORev());
        superstructure = new Superstructure(drive, intake, indexer, shooter);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Replayed robot, disable IO implementations
        // (Use same number of dummy implementations as the real robot)
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        climber = new Climber(new ClimberIOPhoenix());
        indexer = new Indexer(new IndexerIORev());
        intake = new Intake(new IntakeIOPhoenix());
        shooter =
            new Shooter(
                new HoodIOPhoenix(),
                new LauncherIOPhoenix(),
                new TurretIOPhoenix(),
                new KickerIORev());
        superstructure = new Superstructure(drive, intake, indexer, shooter);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "Launcher SysId (Quasistatic Forward)",
    //     shooter.launcherSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Launcher SysId (Quasistatic Reverse)",
    //     shooter.launcherSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Launcher SysId (Dynamic Forward)",
    //     shooter.launcherSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Launcher SysId (Dynamic Reverse)",
    //     shooter.launcherSysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "Kicker SysId (Quasistatic Forward)",
    //     shooter.kickerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Kicker SysId (Quasistatic Reverse)",
    //     shooter.kickerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Kicker SysId (Dynamic Forward)",
    //     shooter.kickerSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Kicker SysId (Dynamic Reverse)",
    //     shooter.kickerSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Climber1 SysId (Quasistatic Forward)",
    //     climber.climber1SysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Climber1 SysId (Quasistatic Reverse)",
    //     climber.climber1SysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Climber1 SysId (Dynamic Forward)",
    //     climber.climber1SysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Climber1 SysId (Dynamic Reverse)",
    //     climber.climber1SysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "Turret SysId (Quasistatic Forward)",
    //     shooter.turretSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Turret SysId (Quasistatic Reverse)",
    //     shooter.turretSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Turret SysId (Dynamic Forward)",
    //     shooter.turretSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Turret SysId (Dynamic Reverse)",
    //     shooter.turretSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* driver */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis() > 0.05));

    // align to trench when right bumper is held
    driverController
        .rightBumper()
        .whileTrue(
            DriveCommands.alignToTrench(
                drive,
                () -> -driverController.getLeftY(), // xsupplier
                () -> -driverController.getLeftX(), // ysupplier
                () -> -driverController.getRightX(),
                () -> driverController.getLeftTriggerAxis() > 0.05));

    // test drive to pose
    // driverController
    //     .povUp()
    //     .whileTrue(
    //         DriveCommands.alignToPose(
    //             drive, AllianceFlipUtil.apply(new Pose2d(2.9, 6.7, new Rotation2d()))));

    // Lock to 45° when B button is held
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtClosest45(
                drive, () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when start button is pressed
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // climb
    driverController
        .povLeft()
        .whileTrue(
            Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
                .andThen(DriveCommands.alignToPose(drive, DriveConstants.leftClimbPos)));
    driverController
        .povRight()
        .whileTrue(
            Commands.defer(() -> drive.pathfindToClimb(false), Set.of(drive))
                .andThen(DriveCommands.alignToPose(drive, DriveConstants.rightClimbPos)));

    // toggle intake mode
    driverController
        .rightTrigger()
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.INTAKE),
                Commands.either(
                    superstructure.setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
                    Commands.either(
                        superstructure.setWantedState(WantedSuperState.SHOOT),
                        superstructure.setWantedState(WantedSuperState.STOPPED),
                        () ->
                            superstructure.getCurrentSuperState()
                                == CurrentSuperState.INTAKING_AND_SHOOTING),
                    () -> superstructure.getCurrentSuperState() == CurrentSuperState.SHOOTING),
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.STOPPED));

    /* operator */

    // manual target
    operatorButtonBox.button(2).onTrue(RobotState.getInstance().toggleManualShooting());
    operatorButtonBox
        .button(3)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.FORWARD));
    operatorButtonBox
        .button(4)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.LEFT_PASS));
    operatorButtonBox.button(5).onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.HUB));
    operatorButtonBox
        .button(6)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.RIGHT_PASS));

    operatorButtonBox.button(7).onTrue(superstructure.setWantedState(WantedSuperState.STOPPED));

    operatorButtonBox.button(11).onTrue(superstructure.setWantedState(WantedSuperState.VOMIT));
    operatorButtonBox.button(11).onFalse(superstructure.setWantedState(WantedSuperState.STOPPED));

    operatorButtonBox
        .button(12)
        .or(driverController.back())
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.SHOOT),
                Commands.either(
                    superstructure.setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
                    Commands.either(
                        superstructure.setWantedState(WantedSuperState.INTAKE),
                        superstructure.setWantedState(WantedSuperState.STOPPED),
                        () ->
                            superstructure.getCurrentSuperState()
                                == CurrentSuperState.INTAKING_AND_SHOOTING),
                    () -> superstructure.getCurrentSuperState() == CurrentSuperState.INTAKING),
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.STOPPED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
