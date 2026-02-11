package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

/** Add your docs here. */
public class AutoFactory {

  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;

  AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
    this.alliance = alliance;
    this.robotContainer = robotContainer;
  }

  public Pair<Pose2d, Command> createMainAuto() {
    var initialPose = RobotState.getInstance().getPose();
    boolean isLeft = RobotState.getInstance().isLeftSide(initialPose);
    return Pair.of(initialPose, Commands.sequence());
  }

  public Pair<Pose2d, Command> createMiddleAuto() {
    var initialPose = RobotState.getInstance().getPose();
    return Pair.of(initialPose, Commands.sequence());
  }
}
