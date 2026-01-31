package autos;

/** Add your docs here. */
public class AutoFactory {

  private final DriverStation.Alliance alliance;

  private final RobotContainer robotContainer;

  Pair<Pose2d, Command> createMainAuto() {
    var initialPose = FieldConstants.getStartingPose(alliance);
    return Pair.of(initialPose, Commands.sequence());
  }

  
}
