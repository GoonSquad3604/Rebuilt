package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class AutoChooser extends SendableChooser<Autos> {

  private static final List<AutoProgram> AUTO_PROGRAMS =
      List.of(
          new AutoProgram(Autos.MAIN, "MAIN", AutoFactory::createMainAuto),
          new AutoProgram(Autos.MIDDLE, "MIDDLE", AutoFactory::createMiddleAuto));

  private final Map<Autos, AutoProgram> programs;
  private final Map<DriverStation.Alliance, Map<Autos, Pair<Pose2d, Command>>> commandCache;
  private final Map<DriverStation.Alliance, AutoFactory> autoFactories;

  private AutoChooser(
      final Map<Autos, AutoProgram> programs,
      final Map<DriverStation.Alliance, AutoFactory> autoFactories) {
    this.programs = programs;
    this.autoFactories = autoFactories;
    commandCache =
        Stream.of(DriverStation.Alliance.values())
            .map(alliance -> Map.entry(alliance, new HashMap<Autos, Pair<Pose2d, Command>>()))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
  }

  /**
   * Create a new <code>AutoChooser</code>
   *
   * @param robotContainer A {@link RobotContainer}
   * @return A new <code>AutoChooser</code> populated with the programs defined in the private field
   *     {@link AutoChooser#AUTO_PROGRAMS}.
   */
  public static AutoChooser create(final RobotContainer robotContainer) {
    var autoFactories =
        Stream.of(DriverStation.Alliance.values())
            .map(alliance -> Map.entry(alliance, new AutoFactory(alliance, robotContainer)))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));
    var programs =
        AUTO_PROGRAMS.stream()
            .map(program -> Map.entry(program.getAuto(), program))
            .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

    var autoChooser = new AutoChooser(programs, autoFactories);

    AUTO_PROGRAMS.forEach(
        program -> {
          if (program.getAuto() == Autos.MAIN) {
            autoChooser.setDefaultOption(program.getLabel(), program.getAuto());
          } else {
            autoChooser.addOption(program.getLabel(), program.getAuto());
          }
        });

    autoChooser.reset(null);

    Shuffleboard.getTab("Autonomous")
        .addString("Selected Auto", () -> autoChooser.getSelected().name())
        .withPosition(12, 3)
        .withSize(6, 2)
        .withWidget(BuiltInWidgets.kTextView);

    return autoChooser;
  }

  /**
   * Update the <code>AutoChooser</code><br>
   * <br>
   *
   * <p>The commands for the selected autonomous program are loaded and cached (if not already
   * present) and the selected program is sent to the {@link
   * edu.wpi.first.wpilibj.shuffleboard.Shuffleboard} under the key <code>Auto/Selected</code>.
   */
  public void update() {
    var selected = getSelected();

    Stream.of(DriverStation.Alliance.values())
        .forEach(
            alliance -> {
              commandCache
                  .get(alliance)
                  .computeIfAbsent(
                      selected,
                      auto ->
                          Pair.of(loadStartingPose(alliance, auto), loadCommand(alliance, auto)));
            });
  }

  /**
   * Reset the caches behind this <code>AutoChooser</code>
   *
   * @param key Optional {@link edu.wpi.first.networktables.NetworkTable} key. If provided the
   *     selected value of the entry at this location will also be reset.
   */
  public void reset(final String key) {
    Stream.of(DriverStation.Alliance.values())
        .forEach(alliance -> commandCache.get(alliance).clear());

    if (key != null) {
      var table = NetworkTableInstance.getDefault().getTable(key);
      table.putValue("selected", NetworkTableValue.makeString("%s".formatted(Autos.MAIN)));
    }
  }

  /**
   * Get the {@link Command} for the selected autonomous program, if available.
   *
   * @return The {@link Command} for the selected autonomous program as an {@link Optional} if
   *     available, otherwise {@link Optional#empty}.
   */
  public Optional<Command> getSelectedCommand() {
    var selected = getSelected();

    return DriverStation.getAlliance()
        .map(
            alliance -> {
              System.out.printf("Running program %s/%s\n", alliance, selected);

              return commandCache.get(alliance).get(selected).getSecond();
            });
  }

  public Optional<Pose2d> getStartingPose() {
    var selected = getSelected();

    return DriverStation.getAlliance()
        .map(
            alliance -> {
              return commandCache.get(alliance).get(selected).getFirst();
            });
  }

  public AutoProgram getProgram() {
    return programs.get(getSelected());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.publishConstString("selected", "%s".formatted(Autos.MAIN));
  }

  private Command loadCommand(final DriverStation.Alliance alliance, final Autos auto) {
    var program = programs.get(auto);

    System.out.printf("Loading command %s/%s\n", alliance, auto);

    return program.getCommand(autoFactories.get(alliance));
  }

  private Pose2d loadStartingPose(final DriverStation.Alliance alliance, final Autos auto) {
    var program = programs.get(auto);

    System.out.printf("Loading command %s/%s\n", alliance, auto);

    return program.getStartingPose(autoFactories.get(alliance));
  }
}
