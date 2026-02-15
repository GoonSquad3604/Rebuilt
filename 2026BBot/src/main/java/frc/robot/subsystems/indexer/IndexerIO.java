package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {

  default void updateInputs(IndexerIOInputs inputs) {}

  @AutoLog
  class IndexerIOInputs {
    // public boolean indexMotorIsConnected = false;

    public double indexVoltage;

    public double indexRPM;

    public double indexCurrent;

    public double indexTemperature;
  }

  public void setIndexPower(double power);

  public void setIndexMotorVoltage(double volts);

  public void setIndexRPM(double RPM);
}
