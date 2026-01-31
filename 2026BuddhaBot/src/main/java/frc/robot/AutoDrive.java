package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;

/** Add your docs here. */
public class AutoDrive {

  @AutoLogOutput private static Pose2d targetPose;

  public static Command alignToPose(Drive drive, Pose2d pose) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              targetPose = pose;
            }),
        Commands.defer(
            () -> drive.pathfindToFieldPose(AllianceFlipUtil.apply(targetPose)), Set.of(drive)));
  }
}
