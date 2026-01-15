// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


public interface IntakeIO {

    class IntakeIOInputs {

        //intaking motor variables
        public double intakeMotorSpeed;
        public double intakeMotorVoltage;
        public double intakeMotorCurrent;

        //arm variables
        public double armPos;
        public double armMotorSpeed;
        public double armMotorVoltage;
        public double armMotorCurrent;

        //arm intake motor variables
        public double armIntakeMotorSpeed;
        public double armIntakeMotorVoltage;
        public double armIntakeMotorCurrent;

        //logging
        public boolean intakeMotorConnected;
        public boolean armIntakeMotorConnected;
        public boolean armMotorConnected;

        //intaking functions
        void setIntakeVoltage(double voltage) {}
        void setIntakePower(double power) {}

        //arm intake motor functions
        void setArmIntakeMotorVoltage(double voltage) {}
        void setArmIntakeMotorPower(double power) {}

        //arm functions
        void setArmPos(double pos){}
        

    }
    
}
