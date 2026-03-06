package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Superstructure.WantedSuperState;

/** Add your docs here. */
public class AutoFactory {

  private final DriverStation.Alliance alliance;
  private final RobotContainer robotContainer;

  AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  //   public Pair<Pose2d, Command> createMainAuto() {
  //     var initialPose = RobotState.getInstance().getPose();
  //     boolean isLeft = RobotState.getInstance().isLeftSide(initialPose);
  //     boolean isMiddle = RobotState.getInstance().isInMiddle(initialPose);
  //     if (isMiddle) {
  //       return Pair.of(
  //           initialPose,
  //           Commands.sequence(
  //               // go to depot
  //               runPath("HubStartToDepot"),

  //               // set intake
  //               robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),

  //               // drive through depot
  //               runPath("ThroughDepot"),

  //               // depot to corral
  //               new ParallelCommandGroup(
  //                   runPath("ThroughDepotToCorral"),
  //                   robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)),

  //               // pick up fuel at corral
  //               new SequentialCommandGroup(
  //                   robotContainer
  //                       .getSuperstructure()
  //                       .setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
  //                   Commands.waitSeconds(1)),

  //               // drive to climb
  //               runPath("CorralToClimb"),

  //               // climb
  //               robotContainer
  //                   .getSuperstructure()
  //                   .setWantedState(WantedSuperState.STOPPED) // replace with climb
  //               ));
  //     } else if (isLeft) {
  //       return Pair.of(
  //           initialPose,
  //           Commands.sequence(
  //               // set intake
  //               robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),

  //               // drive to neutral zone
  //               runPath("StartToNeutralZone"),

  //               // drive to depot while shooting
  //               new ParallelCommandGroup(
  //                   runPath("NeutralZoneToDepot"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(1.5),
  //                       robotContainer
  //                           .getSuperstructure()
  //                           .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),

  //               // pick up depot while shooting
  //               runPath("PickUpDepot"),

  //               // go back to neutral zone
  //               new ParallelCommandGroup(
  //                   runPath("DepotToNeutralZone"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(1.5),
  //
  // robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE))),

  //               // go to corral and shoot on the move
  //               new ParallelCommandGroup(
  //                   runPath("NeutralZoneToCorral"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(1),
  //                       robotContainer
  //                           .getSuperstructure()
  //                           .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),

  //               // wait for feed at corral
  //               Commands.waitSeconds(.5),

  //               // go to climb position
  //               runPath("CorralToClimb"),

  //               // climb
  //               robotContainer
  //                   .getSuperstructure()
  //                   .setWantedState(WantedSuperState.STOPPED) // replace with climb
  //               ));
  //     } else {
  //       return Pair.of(
  //           initialPose,
  //           Commands.sequence(

  //               // set Intake
  //               robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),

  //               // drive to neutral zone
  //               runPath("RightStartToNeutralZone"),

  //               // drive to corral while shooting on the move
  //               new ParallelCommandGroup(
  //                   runPath("RightNeutralZoneToCorral"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(2),
  //
  // robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),

  //               // wait for human players to fill hopper
  //               Commands.waitSeconds(0.5),

  //               // drive to neutral zone set intake after trench
  //               new ParallelCommandGroup(
  //                   runPath("RightCorralToNeutralZone"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(2),
  //
  // robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE))),

  //               // drive to climb with shoot on the move
  //               new ParallelCommandGroup(
  //                   runPath("RightNeutralZoneToClimb"),
  //                   new SequentialCommandGroup(
  //                       Commands.waitSeconds(1),
  //
  // robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),
  //               robotContainer
  //                   .getSuperstructure()
  //                   .setWantedState(WantedSuperState.STOPPED) // change to climb
  //               ));
  //     }
  //   }

  public Pair<Pose2d, Command> createMiddleAuto() {
    var initialPose = RobotState.getInstance().getPose();
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
  }

  public Pair<Pose2d, Command> createLeftTestAuto() {
    var initialPose = RobotState.getInstance().getPose();
    return Pair.of(
        initialPose,
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(5),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("LeftStartNeutralZoneThenDepot"),
                Commands.sequence(
                    Commands.waitSeconds(3.0),
                    robotContainer
                        .getSuperstructure()
                        .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),
            runPath("PickupDepot"),
            runPath("DepotToClimb")));
  }

  private Command runPath(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
