package frc.robot.subsystems.leds;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants;

public class LedsIOCANdle implements LedsIO {

  private CANdle CANdle;

  // animations
  private StrobeAnimation strobe;
  private SolidColor solid;
  private FireAnimation fire;
  private ColorFlowAnimation flow;

  public LedsIOCANdle() {
    CANdle = new CANdle(LedConstants.CANdleID, Constants.CANBusName);

    strobe = new StrobeAnimation(0, LedConstants.STRIP_LENGTH);
    solid = new SolidColor(0, LedConstants.STRIP_LENGTH);
    fire = new FireAnimation(0, LedConstants.STRIP_LENGTH).withFrameRate(15);
    flow = new ColorFlowAnimation(0, LedConstants.STRIP_LENGTH);
  }

  public void updateInputs(LedsIOInputs inputs) {
    inputs.CANdleConnected = CANdle.isConnected();
    inputs.temperature = CANdle.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setColor(RGBWColor color) {
    CANdle.setControl(solid.withColor(color));
  }

  @Override
  public void setStrobe(RGBWColor a, RGBWColor b) {
    CANdle.setControl(strobe.withColor(a).withColor(b));
  }

  @Override
  public void turnOff() {
    CANdle.setControl(new EmptyAnimation(0));
    CANdle.setControl(solid.withColor(LedConstants.black));
  }

  @Override
  public void setRainbow() {
    CANdle.setControl(new RainbowAnimation(0, LedConstants.STRIP_LENGTH));
  }

  @Override
  public void setFire() {
    CANdle.setControl(fire);
  }

  @Override
  public void setStrobe(RGBWColor color) {
    CANdle.setControl(strobe.withColor(color));
  }

  @Override
  public void setFlow(RGBWColor color) {
    CANdle.setControl(flow.withColor(color));
  }
}
