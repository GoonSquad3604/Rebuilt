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

  public Pair<Pose2d, Command> createMainAuto() {
    // var initialPose = RobotState.getInstance().getPose();
    boolean isLeft = RobotState.getInstance().isLeftSide();
    boolean isMiddle = RobotState.getInstance().isInMiddle();
    if (isLeft) {
      // left auto
      return Pair.of(
          RobotState.getInstance().getPose(),
          Commands.sequence(
              robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
              Commands.parallel(
                  runPath("LeftStartToNeutral"),
                  Commands.sequence(
                      Commands.waitSeconds(5.2),
                      robotContainer
                          .getSuperstructure()
                          .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),
              runPath("LeftPickUpDepot"),
              Commands.waitSeconds(5),
              robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
              Commands.waitSeconds(5),
              // remove later
              robotContainer.getSuperstructure().setWantedState(WantedSuperState.STOPPED)

              // robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_LEFT)

              ));
    } else if (isMiddle) {
      // middle auto
      return Pair.of(
          RobotState.getInstance().getPose(),
          Commands.sequence(
              robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
              runPath("MiddleGoToDepot"),
              runPath("LeftPickUpDepot"),
              Commands.waitSeconds(10),
              // remove later
              robotContainer.getSuperstructure().setWantedState(WantedSuperState.STOPPED)));
    } else {
      // right auto
      return Pair.of(
          RobotState.getInstance().getPose(),
          Commands.sequence(
              // robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
              ));
    }
  }

  public Pair<Pose2d, Command> createMiddleAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("MiddleGoToDepot"),
            runPath("MiddlePickUpDepot"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(8.5),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("LeftClimb"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  public Pair<Pose2d, Command> createLeftAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("LeftStartToNeutralToClimb"),
                Commands.sequence(
                    Commands.waitSeconds(5.6),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),
            Commands.waitSeconds(10),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("LeftClimb"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  public Pair<Pose2d, Command> createRightAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("RightStartToNeutralZone"),
                Commands.sequence(
                    Commands.waitSeconds(5.0),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),
            Commands.waitSeconds(8),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("RightClimb"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  public Pair<Pose2d, Command> createRightNoClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("RightStartToNeutralZoneNoClimb"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(7),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("RightNoClimbPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
  }

  public Pair<Pose2d, Command> createLeftNoClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("LeftStartToNeutralZone"),
                Commands.sequence(
                    Commands.waitSeconds(6),
                    robotContainer
                        .getSuperstructure()
                        .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),
            runPath("LeftPickUpDepotNoClimb"),
            Commands.waitSeconds(6),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
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
