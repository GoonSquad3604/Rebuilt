package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    public static final double zOffset = 36;
    public static final double xOffset = 11.5;
    public static final double yOffset = 0;

    public static Transform3d robotToTurret = new Transform3d(Units.inchesToMeters(11.5), Units.inchesToMeters(0.0), Units.inchesToMeters(36), Rotation3d.kZero);

}
