package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;

import frc.robot.subsystems.led.LED.AnimationType;

public interface LEDIO {

  default void setLEDs(RGBWColor color) {}

  default void setAnimation(AnimationType animation) {}

  default void clearAnimation() {}
}
