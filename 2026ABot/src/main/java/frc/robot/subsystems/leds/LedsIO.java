package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.RGBWColor;
import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {

  @AutoLog
  class LedsIOInputs {
    public boolean CANdleConnected = false;
    // public double voltage;
    // public double current;
    public double temperature;
  }

  default void updateInputs(LedsIOInputs inputs) {}

  default void setColor(RGBWColor color) {}

  default void turnOff() {}

  default void setRainbow() {}

  default void setFire() {}

  default void setStrobe(RGBWColor color) {}

  default void setStrobe(RGBWColor a, RGBWColor b) {}

  default void setFlow(RGBWColor color) {}
}
