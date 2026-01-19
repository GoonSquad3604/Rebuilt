package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.subsystems.led.LED.AnimationType;

public class LEDIOCANdle implements LEDIO {

  private final CANdle candle;

  public LEDIOCANdle(int port, String canBus) {
    this.candle = new CANdle(port, canBus);
    var cfg = new CANdleConfiguration();
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.5;
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(cfg);
  }

  public void setLEDs(RGBWColor color) {
    candle.setControl(new SolidColor(0, LEDConstants.NUMBER_OF_LEDS).withColor(color));
  }

  @Override
  public void setAnimation(AnimationType animation) {
    switch (animation) {
      case IDLE:
        candle.setControl(
            new LarsonAnimation(0, LEDConstants.NUMBER_OF_LEDS)
                .withSlot(0)
                .withColor(LEDConstants.kViolet));
        break;
      case RAINBOW:
        candle.setControl(new RainbowAnimation(0, LEDConstants.NUMBER_OF_LEDS).withSlot(0));
        break;
      case STROBE_CORRAL:
        candle.setControl(
            new StrobeAnimation(0, LEDConstants.NUMBER_OF_LEDS)
                .withSlot(0)
                .withColor(LEDConstants.kYellow));
        break;
      case STROBE_FORWARD:
        candle.setControl(
            new StrobeAnimation(0, LEDConstants.NUMBER_OF_LEDS)
                .withSlot(0)
                .withColor(LEDConstants.kWhite));
        break;
      case STROBE_HUB:
        candle.setControl(
            new StrobeAnimation(0, LEDConstants.NUMBER_OF_LEDS)
                .withSlot(0)
                .withColor(LEDConstants.kRed));
        break;
      case STROBE_ZONE:
        candle.setControl(
            new StrobeAnimation(0, LEDConstants.NUMBER_OF_LEDS)
                .withSlot(0)
                .withColor(LEDConstants.kOrange));
        break;
    }
  }

  @Override
  public void clearAnimation() {
    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
  }
}
