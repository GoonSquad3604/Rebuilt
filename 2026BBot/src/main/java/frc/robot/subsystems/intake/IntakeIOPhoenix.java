// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


public class IntakeIOPhoenix implements IntakeIO{

        //intaking motor variables
        public double intakeMotorSpeed;
        public double intakeMotorVoltage;
        public double intakeMotorCurrent;

        //arm functions
        public double armPos;
        public double armMotorVoltage;
        public double armMotorCurrent;

        //logging
        public boolean intakeMotorConnected;
        public boolean armMotorConnected;
        
        //intaking functions
        void setVoltage(double voltage) {}
        void setPower(double power) {}

        //arm functions
        void setArmPos(double pos){}
}
