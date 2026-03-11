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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState.ShooterTarget;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.CurrentSuperState;
import frc.robot.subsystems.Superstructure.WantedSuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOPhoenix;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOPhoenix;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.hinge.HingeIOPhoenix;
import frc.robot.subsystems.intake.rollers.RollerSystemIOPhoenix;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOPhoenix;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.hood.HoodIOPhoenix;
import frc.robot.subsystems.shooter.launcher.LauncherIOPhoenix;
import frc.robot.subsystems.shooter.turret.TurretIOPhoenix;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOPhoenix;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Hopper hopper;
  private final Intake intake;
  private final Kicker kicker;
  private final Shooter shooter;
  private final Spindexer spindexer;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  private final CommandXboxController testController = new CommandXboxController(2);
  private final CommandJoystick pitBox = new CommandJoystick(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        hopper = new Hopper(new HopperIOPhoenix());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    camera0Name, robotToCamera0Retracted, robotToCamera0Extended, hopper),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));
        climber = new Climber(new ClimberIOPhoenix());
        intake = new Intake(new RollerSystemIOPhoenix(), new HingeIOPhoenix());
        kicker = new Kicker(new KickerIOPhoenix());
        shooter = new Shooter(new HoodIOPhoenix(), new LauncherIOPhoenix(), new TurretIOPhoenix());
        spindexer = new Spindexer(new SpindexerIOPhoenix());
        superstructure =
            new Superstructure(drive, climber, hopper, intake, kicker, shooter, spindexer);

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

        hopper = new Hopper(new HopperIOPhoenix());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0Retracted, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));
        climber = new Climber(new ClimberIOPhoenix());
        intake = new Intake(new RollerSystemIOPhoenix(), new HingeIOPhoenix());
        kicker = new Kicker(new KickerIOPhoenix());
        shooter = new Shooter(new HoodIOPhoenix(), new LauncherIOPhoenix(), new TurretIOPhoenix());
        spindexer = new Spindexer(new SpindexerIOPhoenix());
        superstructure =
            new Superstructure(drive, climber, hopper, intake, kicker, shooter, spindexer);
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
        hopper = new Hopper(new HopperIOPhoenix());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0Retracted, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));

        climber = new Climber(new ClimberIOPhoenix());
        intake = new Intake(new RollerSystemIOPhoenix(), new HingeIOPhoenix());
        kicker = new Kicker(new KickerIOPhoenix());
        shooter = new Shooter(new HoodIOPhoenix(), new LauncherIOPhoenix(), new TurretIOPhoenix());
        spindexer = new Spindexer(new SpindexerIOPhoenix());
        superstructure =
            new Superstructure(drive, climber, hopper, intake, kicker, shooter, spindexer);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    autoChooser.addOption(
        "Spindexer SysId (Quasistatic Forward)",
        spindexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Spindexer SysId (Quasistatic Reverse)",
        spindexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Spindexer SysId (Dynamic Forward)",
        spindexer.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Spindexer SysId (Dynamic Reverse)",
        spindexer.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption(
    //     "Kicker SysId (Quasistatic Forward)",
    //     kicker.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Kicker SysId (Quasistatic Reverse)",
    //     kicker.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Kicker SysId (Dynamic Forward)", kicker.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Kicker SysId (Dynamic Reverse)", kicker.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // driverController
    //     .rightBumper()
    //     .whileTrue(
    //         DriveCommands.alignToTrench(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> -driverController.getRightX(),
    //             () -> driverController.getLeftTriggerAxis() > 0.05));

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
    // driverController
    //     .povLeft()
    //     .and(() -> superstructure.getCurrentSuperState() == CurrentSuperState.CLIMBING)
    //     .whileTrue(
    //         Commands.defer(() -> drive.pathfindToClimb(true), Set.of(drive))
    //             .andThen(DriveCommands.alignToPose(drive, DriveConstants.leftClimbPos)));
    // driverController
    //     .povRight()
    //     .and(() -> superstructure.getCurrentSuperState() == CurrentSuperState.CLIMBING)
    //     .whileTrue(
    //         Commands.defer(() -> drive.pathfindToClimb(false), Set.of(drive))
    //             .andThen(DriveCommands.alignToPose(drive, DriveConstants.rightClimbPos)));

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

    // hopper in
    driverController.leftBumper().onTrue(Commands.runOnce(() -> hopper.setPower(0.15)));
    driverController.leftBumper().onFalse(Commands.runOnce(() -> hopper.setPower(0.0)));

    driverController.rightBumper().onTrue(Commands.runOnce(() -> hopper.setPower(-0.15)));
    driverController.rightBumper().onFalse(Commands.runOnce(() -> hopper.setPower(0.0)));

    // reset hopper
    driverController.a().onTrue(Commands.runOnce(() -> hopper.zeroEncoder()));

    // cllimber testing
    // driverController.povUp().onTrue(Commands.runOnce(() -> climber.setPowerInnerRungs(-1)));
    // driverController.povUp().onFalse(Commands.runOnce(() -> climber.setPowerInnerRungs(0.0)));

    // driverController.povDown().onTrue(Commands.runOnce(() -> climber.setPowerInnerRungs(1)));
    // driverController.povDown().onFalse(Commands.runOnce(() -> climber.setPowerInnerRungs(0.0)));

    // driverController.y().onTrue(Commands.runOnce(() -> climber.setPowerOuterRungs(1)));
    // driverController.y().onFalse(Commands.runOnce(() -> climber.setPowerOuterRungs(0.0)));

    // driverController.a().onTrue(Commands.runOnce(() -> climber.setPowerOuterRungs(-1)));
    // driverController.a().onFalse(Commands.runOnce(() -> climber.setPowerOuterRungs(0.0)));

    /* operator */

    // operatorButtonBox.button(10).onTrue(superstructure.setWantedState(WantedSuperState.CLIMB));

    // manual target
    operatorButtonBox.button(1).onTrue(RobotState.getInstance().toggleManualShooting());
    operatorButtonBox
        .button(2)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.LEFT_PASS));
    operatorButtonBox.button(3).onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.HUB));
    operatorButtonBox
        .button(4)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.RIGHT_PASS));
    operatorButtonBox
        .button(5)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.FORWARD));

    // manual climb
    operatorButtonBox.button(6).onTrue(Commands.runOnce(() -> climber.progressManualClimb()));

    // reset
    operatorButtonBox
        .button(8)
        .onTrue(
            superstructure
                .setWantedState(WantedSuperState.STOPPED)
                .andThen(Commands.runOnce(() -> climber.setAutoClimbing(false))));

    // stow intake
    operatorButtonBox.button(7).onTrue(superstructure.setWantedState(WantedSuperState.STOW));

    // set climb mode
    operatorButtonBox.button(10).onTrue(superstructure.setWantedState(WantedSuperState.CLIMB));

    // toggle shoot mode
    operatorButtonBox
        .button(11)
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

    // test buttons:
    // kicker and shooter
    testController
        .b()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(() -> kicker.setPower(0.5)),
                Commands.runOnce(() -> shooter.setLauncherPower(-0.5))));

    testController
        .b()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> kicker.setPower(0)),
                Commands.runOnce(() -> shooter.setLauncherPower(0))));

    // spindex
    // pitBox
    //     .button(4)
    //     .onTrue(
    //         Commands.runOnce(() -> shooter.setLauncherVelocity(40))
    //             .alongWith(Commands.runOnce(() -> kicker.setVelocity(40))));
    // pitBox
    //     .button(4)
    //     .onFalse(
    //         Commands.runOnce(() -> shooter.setLauncherPower(0.0))
    //             .alongWith(Commands.runOnce(() -> kicker.setPower(0.0))));

    pitBox.button(5).onTrue(Commands.runOnce(() -> spindexer.setPower(0.6)));
    pitBox.button(5).onFalse(Commands.runOnce(() -> spindexer.setPower(0.0)));

    // turret
    pitBox.button(6).onTrue(Commands.runOnce(() -> shooter.setTurretPower(0.15)));
    pitBox.button(6).onFalse(Commands.runOnce(() -> shooter.setTurretPower(0.0)));
    pitBox.button(7).onTrue(Commands.runOnce(() -> shooter.setTurretPower(-0.15)));
    pitBox.button(7).onFalse(Commands.runOnce(() -> shooter.setTurretPower(0.0)));

    // hood
    pitBox.button(8).onTrue(Commands.runOnce(() -> shooter.setHoodPower(0.07)));
    pitBox.button(8).onFalse(Commands.runOnce(() -> shooter.setHoodPower(0.0)));
    pitBox.button(9).onTrue(Commands.runOnce(() -> shooter.setHoodPower(-0.07)));
    pitBox.button(9).onFalse(Commands.runOnce(() -> shooter.setHoodPower(0.0)));

    pitBox
        .button(1)
        .onTrue(
            Commands.runOnce(
                () -> shooter.setHoodPosition(ShooterConstants.HoodConstants.hoodMaxPos)));

    pitBox
        .button(2)
        .onTrue(
            Commands.runOnce(
                () -> shooter.setHoodPosition(ShooterConstants.HoodConstants.hoodMinPos)));

    pitBox.button(3).onTrue(Commands.runOnce(() -> shooter.setHoodPositionDashboard()));

    pitBox
        .button(4)
        .onTrue(Commands.runOnce(() -> spindexer.setVelocity(SpindexerConstants.spinVelocity)));
    pitBox.button(4).onFalse(Commands.runOnce(() -> spindexer.setPower(0)));

    pitBox
        .button(5)
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.STOPPED),
                superstructure.setWantedState(WantedSuperState.EJECT),
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.EJECTING));

    pitBox.button(10).onTrue(Commands.runOnce(() -> shooter.setTurretPos(0.5)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
