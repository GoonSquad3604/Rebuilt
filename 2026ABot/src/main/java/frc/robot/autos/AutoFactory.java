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

  public AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  public Pair<Pose2d, Command> createLeftClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("LeftClimbPt1"),
                Commands.sequence(
                    Commands.waitSeconds(5.7),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),
            Commands.waitSeconds(5.0),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("LeftClimbPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  // unused
  public Pair<Pose2d, Command> createLeftDepotAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.waitSeconds(0.3604),
            Commands.parallel(
                runPath("LeftDepotPt1"),
                Commands.sequence(
                    Commands.waitSeconds(6),
                    robotContainer
                        .getSuperstructure()
                        .setWantedState(WantedSuperState.INTAKE_AND_SHOOT))),
            runPath("LeftDepotPt2"),
            Commands.waitSeconds(6),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
  }

  // unused
  public Pair<Pose2d, Command> createLeftDoubleSwipeAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.waitSeconds(0.3604),
            runPath("LeftDoubleSwipePt1"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(6.75),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("LeftDoubleSwipePt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
  }

  public Pair<Pose2d, Command> createLeftDoubleSwipeBumpAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("LeftDoubleSwipeBumpPt1New"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            runPath("LeftDoubleSwipeBumpPt2"),
            Commands.waitSeconds(2.0),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("LeftDoubleSwipeBumpPt3"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            runPath("LeftDoubleSwipeBumpPt4")));
  }

  public Pair<Pose2d, Command> createRightClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("RightClimbPt1"),
                Commands.sequence(
                    Commands.waitSeconds(4.8),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT))),
            Commands.waitSeconds(5.5),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("RightClimbPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  // unused
  public Pair<Pose2d, Command> createRightDoubleSwipeAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.waitSeconds(0.3604),
            runPath("RightDoubleSwipePt1"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(6.5),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("RightDoubleSwipePt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT)));
  }

  public Pair<Pose2d, Command> createRightDoubleSwipeBumpAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("RightDoubleSwipeBumpPt1"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            runPath("RightDoubleSwipeBumpPt2"),
            Commands.waitSeconds(2.0),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("RightDoubleSwipeBumpPt3"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            runPath("RightDoubleSwipeBumpPt4")));
  }

  public Pair<Pose2d, Command> createMiddleDepotClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
            runPath("MiddleDepotClimbPt1"),
            runPath("MiddleDepotClimbPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(6.5),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("MiddleDepotClimbPt3"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  public Pair<Pose2d, Command> createMiddleDepotNeutralAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            runPath("MiddleDepotNeutralPt1"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
            Commands.parallel(
                runPath("MiddleDepotNeutralPt2"),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
                    Commands.waitSeconds(3.5),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE))),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            runPath("MiddleDepotNeutralPt3")));
  }

  // unused
  public Pair<Pose2d, Command> createRightPassAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
            Commands.parallel(
                runPath("RightPass"),
                Commands.sequence(
                    Commands.waitSeconds(1),
                    robotContainer
                        .getSuperstructure()
                        .setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
                    Commands.waitUntil(() -> RobotState.getInstance().nearTrench()),
                    robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE),
                    Commands.waitUntil(() -> !RobotState.getInstance().nearTrench()),
                    robotContainer
                        .getSuperstructure()
                        .setWantedState(WantedSuperState.INTAKE_AND_SHOOT)))));
  }

  // hub starting pose variant of middle auto
  public Pair<Pose2d, Command> createHubMiddleDepotClimbAuto() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.INTAKE_AND_SHOOT),
            runPath("HubMiddleDepotClimbPt1"),
            runPath("HubMiddleDepotClimbPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SHOOT),
            Commands.waitSeconds(6),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("MiddleDepotClimbPt3"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
  }

  // very used (trust me)
  public Pair<Pose2d, Command> createChaos_hehe() {
    return Pair.of(
        RobotState.getInstance().getPose(),
        Commands.sequence(
            Commands.waitSeconds(4),
            runPath("ChaosPt1"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.SET_UP_AUTO_CLIMB),
            Commands.waitUntil(() -> robotContainer.getSuperstructure().climberDeployed()),
            runPath("ChaosPt2"),
            robotContainer.getSuperstructure().setWantedState(WantedSuperState.CLIMB_IN_AUTO)));
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
