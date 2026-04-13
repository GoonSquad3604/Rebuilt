package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.signals.RGBWColor;
import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {

  @AutoLog
  class LEDIOInputs {
    public boolean CANdleConnected = false;
    public double voltage;
    public double current;
    public double temperature;
  }

  default void updateInputs(LEDIOInputs inputs) {}

  default void setColor(RGBWColor color) {}

  default void turnOff() {}

  default void setRainbow() {}

  default void setFire() {}

  default void setStrobe() {}
}
