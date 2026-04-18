package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;

public class LedConstants {

  public static final int CANdleID = 58;

  public static final int STRIP_LENGTH = 56;

  public static final double colorSwitchInterval = 0.3604;

  // colors
  public static final RGBWColor white = new RGBWColor(100, 100, 100, 150);
  public static final RGBWColor black = new RGBWColor(0, 0, 0, 150);
  public static final RGBWColor purple = new RGBWColor(71, 10, 87, 150);
  public static final RGBWColor red = new RGBWColor(100, 0, 0, 150);
  public static final RGBWColor orange = new RGBWColor(100, 20, 0);
  public static final RGBWColor blue = new RGBWColor(0, 0, 100, 150);
  public static final RGBWColor green = new RGBWColor(0, 100, 0, 150);
}
