// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.climber.Climber;
// import frc.robot.subsystems.climber.Climber.ClimberWantedState;

// public class AutomatedClimb extends SequentialCommandGroup {
//   /** Creates a new AutomatedClimb. */
//   public AutomatedClimb(Climber climber) {
//     addCommands(
//         Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.DEPLOY)),
//         Commands.waitUntil(() -> climber.canProceed()),

//         // Commands.waitUntil(() -> climber.getCurrentState() == ClimberCurrentState.DEPLOYED),
//         // Commands.waitUntil(
//         //     () ->
//         //         RobotState.getInstance().atDrivePosition(DriveConstants.leftClimbPos)
//         //             ||
// RobotState.getInstance().atDrivePosition(DriveConstants.rightClimbPos)),
//         // Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_LOW_RUNG)),
//         Commands.runOnce(() -> climber.addClimbStep()),
//         Commands.waitUntil(() -> climber.canProceed()),
//         // Commands.waitUntil(() -> climber.getCurrentState() ==
// ClimberCurrentState.ON_LOW_RUNG),
//         // Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.GRAB_MID_RUNG)),
//         // Commands.waitUntil(() -> climber.getCurrentState() ==
//         // ClimberCurrentState.GRABBED_MID_RUNG),
//         // Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.DEPLOY_OUTER)),
//         // Commands.waitUntil(() -> climber.getCurrentState() ==
//         // ClimberCurrentState.DEPLOYED_OUTER),
//         // Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_MID_RUNG)),
//         Commands.runOnce(() -> climber.addClimbStep()),
//         Commands.waitUntil(() -> climber.canProceed()),
//         // Commands.waitUntil(() -> climber.getCurrentState() ==
// ClimberCurrentState.ON_MID_RUNG),
//         // Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.GRAB_HIGH_RUNG)),
//         Commands.runOnce(() -> climber.addClimbStep()),
//         Commands.waitUntil(() -> climber.canProceed()),
//         // Commands.waitUntil(
//         //     () -> climber.getCurrentState() == ClimberCurrentState.GRABBED_HIGH_RUNG),
//         Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.RELEASE_INNER)),
//         Commands.runOnce(() -> climber.addClimbStep()),
//         Commands.waitUntil(() -> climber.canProceed()),
//         // Commands.waitUntil(() -> climber.getCurrentState() ==
//         // ClimberCurrentState.RELEASED_INNER),
//         Commands.runOnce(() -> climber.setWantedState(ClimberWantedState.CLIMB_HIGH_RUNG)));

//     // climber.setAutoClimbing(false);
//   }
// }
