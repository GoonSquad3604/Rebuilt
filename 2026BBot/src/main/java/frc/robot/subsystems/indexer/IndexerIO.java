// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {
    

    default void updateInputs(IndexerIOInputs inputs) {}

    @AutoLog
    class IndexerIOInputs{
        public boolean indexerIsConnected = false;
        public double indexerVoltage;
        public double indexerCurrent;
        public double indexerSpeed;
        public double indexerTemperature;
    }
    public void setIndexPower(double power);
    public void setVoltage(double volts);
}
