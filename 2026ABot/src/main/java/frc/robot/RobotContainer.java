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
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOPhoenix;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.hinge.HingeIOPhoenix;
import frc.robot.subsystems.intake.rollers.RollerSystemIOPhoenix;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerIOPhoenix;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIOCANdle;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.hood.HoodIOPhoenix;
import frc.robot.subsystems.shooter.launcher.LauncherIOPhoenix;
import frc.robot.subsystems.shooter.turret.TurretIOPhoenix;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerIOPhoenix;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
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
  private final Leds leds;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandJoystick operatorButtonBox = new CommandJoystick(1);
  private final CommandXboxController testController = new CommandXboxController(2);
  //   private final CommandJoystick pitBox = new CommandJoystick(3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
            new VisionIOPhotonVision(camera0Name, robotToCamera0),
            new VisionIOPhotonVision(camera1Name, robotToCamera1),
            new VisionIOPhotonVision(camera2Name, robotToCamera2),
            new VisionIOPhotonVision(camera3Name, robotToCamera3));
    climber = new Climber(new ClimberIOPhoenix());
    intake = new Intake(new RollerSystemIOPhoenix(), new HingeIOPhoenix());
    kicker = new Kicker(new KickerIOPhoenix());
    shooter = new Shooter(new HoodIOPhoenix(), new LauncherIOPhoenix(), new TurretIOPhoenix());
    spindexer = new Spindexer(new SpindexerIOPhoenix());
    leds = new Leds(new LedsIOCANdle());
    superstructure =
        new Superstructure(drive, climber, hopper, intake, kicker, shooter, spindexer, leds);

    // Set up SysId auto chooser
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

    // autoChooser.addOption(
    //     "Spindexer SysId (Quasistatic Forward)",
    //     spindexer.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Spindexer SysId (Quasistatic Reverse)",
    //     spindexer.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Spindexer SysId (Dynamic Forward)",
    //     spindexer.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Spindexer SysId (Dynamic Reverse)",
    //     spindexer.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    /* DRIVER */

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis() > 0.05));

    // Align to trench when right bumper is held
    driverController
        .rightBumper()
        .whileTrue(
            DriveCommands.alignToTrench(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX(),
                    () -> driverController.getLeftTriggerAxis() > 0.05)
                .alongWith(superstructure.setWantedState(WantedSuperState.TRACK)));

    // Lock to 45° when B button is held
    driverController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtClosest45(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> driverController.getLeftTriggerAxis() > 0.05));

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

    // Toggle intake mode
    driverController
        .rightTrigger()
        .or(testController.rightTrigger())
        .onTrue(
            Commands.either(
                    superstructure.setWantedState(WantedSuperState.SHOOT),
                    Commands.either(
                        superstructure.setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
                        Commands.either(
                            superstructure.setWantedState(WantedSuperState.INTAKE),
                            superstructure.setWantedState(WantedSuperState.TRACK),
                            () ->
                                superstructure.getCurrentSuperState()
                                    != CurrentSuperState.INTAKING),
                        () -> superstructure.getCurrentSuperState() == CurrentSuperState.SHOOTING),
                    () ->
                        superstructure.getCurrentSuperState()
                            == CurrentSuperState.INTAKING_AND_SHOOTING)
                .ignoringDisable(true));

    // toggle shoot mode
    driverController
        .leftBumper()
        .or(testController.leftBumper())
        .onTrue(
            Commands.either(
                    superstructure.setWantedState(WantedSuperState.INTAKE),
                    Commands.either(
                        superstructure.setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
                        Commands.either(
                            superstructure.setWantedState(WantedSuperState.TRACK),
                            superstructure.setWantedState(WantedSuperState.SHOOT),
                            () ->
                                superstructure.getCurrentSuperState()
                                    == CurrentSuperState.SHOOTING),
                        () -> superstructure.getCurrentSuperState() == CurrentSuperState.INTAKING),
                    () ->
                        superstructure.getCurrentSuperState()
                            == CurrentSuperState.INTAKING_AND_SHOOTING)
                .ignoringDisable(true));

    // Toggle ready to climb
    driverController
        .povUp()
        .and(() -> superstructure.getCurrentSuperState() == CurrentSuperState.CLIMBING)
        .onTrue(superstructure.toggleReadyToClimb());

    // toggle vomit mode
    driverController
        .povDown()
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.EJECT),
                superstructure.setWantedState(WantedSuperState.TRACK),
                () -> superstructure.getCurrentSuperState() != CurrentSuperState.EJECTING));

    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtClimbRotation(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> driverController.getLeftTriggerAxis() > 0.05));

    /* OPERATOR */

    // Toggle manual targeting
    operatorButtonBox
        .button(1)
        .onTrue(RobotState.getInstance().toggleManualShooting().ignoringDisable(true));

    // Set manual target to left pass
    operatorButtonBox
        .button(2)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.LEFT_PASS));

    // Eject
    operatorButtonBox
        .button(3)
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.STOPPED),
                superstructure.setWantedState(WantedSuperState.EJECT),
                () -> superstructure.getCurrentSuperState() == CurrentSuperState.EJECTING));

    // Set manual target to right pass
    operatorButtonBox
        .button(4)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.RIGHT_PASS));

    // Set manual target to forward
    operatorButtonBox
        .button(5)
        .onTrue(RobotState.getInstance().setManualTarget(ShooterTarget.FORWARD));

    // Stow intake/hopper
    operatorButtonBox
        .button(6)
        .onTrue(superstructure.setWantedState(WantedSuperState.STOW).ignoringDisable(true));

    // Progress Manual Climb
    operatorButtonBox
        .button(7)
        .and(() -> hopper.isStowed())
        .onTrue(Commands.runOnce(() -> climber.progressManualClimb()));

    // Reset
    operatorButtonBox
        .button(8)
        .onTrue(
            superstructure
                .setWantedState(WantedSuperState.STOPPED)
                .ignoringDisable(true)
                .andThen(Commands.runOnce(() -> climber.resetClimbStep()))
                .ignoringDisable(true));

    // Declimb
    operatorButtonBox
        .button(9)
        .onTrue(superstructure.setWantedState(WantedSuperState.DECLIMB).ignoringDisable(true));

    // Climb
    operatorButtonBox
        .button(10)
        .onTrue(
            Commands.either(
                    superstructure.setWantedState(WantedSuperState.CLIMB),
                    superstructure.setWantedState(WantedSuperState.STOPPED),
                    () -> superstructure.getCurrentSuperState() != CurrentSuperState.CLIMBING)
                .ignoringDisable(true));

    // Unjam
    operatorButtonBox.button(11).onTrue(superstructure.setWantedState(WantedSuperState.UNJAM));
    operatorButtonBox.button(11).onFalse(superstructure.setWantedState(WantedSuperState.TRACK));

    // Clean (only available on pit box)
    operatorButtonBox
        .button(12)
        .or(testController.x())
        .onTrue(
            Commands.either(
                    superstructure.setWantedState(WantedSuperState.CLEAN),
                    superstructure.setWantedState(WantedSuperState.STOPPED),
                    () -> superstructure.getCurrentSuperState() != CurrentSuperState.CLEANING)
                .ignoringDisable(true));

    /* test controller */

    testController.povUp().onTrue(Commands.runOnce(() -> climber.setPowerInnerRungs(-.8)));
    testController.povUp().onFalse(Commands.runOnce(() -> climber.setPowerInnerRungs(0.0)));

    testController.povDown().onTrue(Commands.runOnce(() -> climber.setPowerInnerRungs(.8)));
    testController.povDown().onFalse(Commands.runOnce(() -> climber.setPowerInnerRungs(0.0)));

    testController.y().onTrue(Commands.runOnce(() -> climber.setPowerOuterRungs(.8)));
    testController.y().onFalse(Commands.runOnce(() -> climber.setPowerOuterRungs(0.0)));

    testController.a().onTrue(Commands.runOnce(() -> climber.setPowerOuterRungs(-.8)));
    testController.a().onFalse(Commands.runOnce(() -> climber.setPowerOuterRungs(0.0)));

    testController
        .b()
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.TRACK),
                superstructure.setWantedState(WantedSuperState.STOPPED),
                () -> superstructure.getCurrentSuperState() != CurrentSuperState.TRACKING));

    driverController
        .rightStick()
        .onTrue(
            Commands.either(
                superstructure.setWantedState(WantedSuperState.TEST_SHOOT),
                superstructure.setWantedState(WantedSuperState.TRACK),
                () -> superstructure.getCurrentSuperState() != CurrentSuperState.TESTING_SHOOTING));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getSysIdRoutine() {
    return autoChooser.get();
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }

  public Hopper getHopper() {
    return hopper;
  }
}
