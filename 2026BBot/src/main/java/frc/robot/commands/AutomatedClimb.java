package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberCurrentState;
import frc.robot.subsystems.climber.Climber.ClimberWantedState;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AutomatedClimb extends SequentialCommandGroup {
  /** Creates a new AutomatedClimb. */
  public AutomatedClimb(Climber climber) {
    Logger.recordOutput("Subsystems/Climber/BeganAutoClimb", true);
    addCommands(
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.DEPLOY)),
        Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.DEPLOYED),
        Commands.waitUntil(
            () ->
                RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)
                    || RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_LOW_RUNG)),
        Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.ON_LOW_RUNG),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.GRAB_MID_RUNG)),
        Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.GRABBED_MID_RUNG),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.DEPLOY_OUTER)),
        Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.DEPLOYED_OUTER),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_MID_RUNG)),
        Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.ON_MID_RUNG),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.GRAB_HIGH_RUNG)),
        Commands.waitUntil(
            () -> climber.getCurrentState() == ClimberCurrentState.GRABBED_HIGH_RUNG),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.RELEASE_INNER)),
        Commands.waitUntil(
            () ->
                climber.getCurrentState() == ClimberCurrentState.GRABBING_HIGH_RUNG_INNER_RELEASED),
        Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_HIGH_RUNG)));

    climber.toggleIsNotAutoClimbing();
  }
}
