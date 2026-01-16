package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

public class LEDConstants {

  public static final int NUMBER_OF_LEDS = 58; // CHANGE LATER

  // colors
  public static final RGBWColor kBlack = new RGBWColor(0, 0, 0, 0);
  public static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
  public static final RGBWColor kViolet = RGBWColor.fromHex("#470a57").orElseThrow();
  public static final RGBWColor kYellow = RGBWColor.fromHex("#f8fc03").orElseThrow();
  public static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
  public static final RGBWColor kWhite = RGBWColor.fromHex("#ffffff").orElseThrow();
  public static final RGBWColor kOrange = RGBWColor.fromHex("#fc7f03").orElseThrow();
  
}
