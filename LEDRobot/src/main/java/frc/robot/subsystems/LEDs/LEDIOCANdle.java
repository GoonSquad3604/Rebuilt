package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.LEDs.LEDIO.LEDIOInputs;

public class LEDIOCANdle implements LEDIO {

  private CANdle leds;
  private boolean animated = false;
  private LEDConstants constants = new LEDConstants();

  // animations
  private StrobeAnimation strobe = new StrobeAnimation(0, 399);

  private SolidColor solid = new SolidColor(0, 399);

  private RainbowAnimation rainbow = new RainbowAnimation(0, 399);

  private FireAnimation fire = new FireAnimation(0, 399);

  private ColorFlowAnimation flow = new ColorFlowAnimation(0, 399);

  public LEDIOCANdle() {
    leds = new CANdle(constants.CANdleID);
    leds.setControl(flow.withColor(constants.purple));
  }

  public void updateInputs(LEDIOInputs inputs) {
    inputs.CANdleConnected = leds.isConnected();
    inputs.temperature = leds.getDeviceTemp().getValueAsDouble();
    inputs.voltage = leds.getSupplyVoltage().getValueAsDouble();
    inputs.current = leds.getOutputCurrent().getValueAsDouble();
  }

  public void setColor(RGBWColor color) {
    animated = false;
    leds.setControl(solid.withColor(color));
  }

  public void turnOff() {
    if (animated) leds.setControl(new EmptyAnimation(0));
    else leds.setControl(solid.withColor(constants.black));
  }

  public void setRainbow() {
    animated = true;
    leds.setControl(rainbow);
  }

  public void setFire() {
    animated = true;
    leds.setControl(fire);
  }

  public void setStrobe(RGBWColor color) {
    animated = true;
    leds.setControl(strobe.withColor(color));
  }
}
