/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.koibots.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    @Override
    public void robotInit() {
        // initialize motor
        motor1 = new CANSparkMax(4, MotorType.kBrushed);
        motor2 = new CANSparkMax(5, MotorType.kBrushed);

        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();

        motor1.setIdleMode(IdleMode.kCoast);
        motor2.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void autonomousPeriodic() {
        motor1.set(.95);
        motor2.set(.95);
    }
    
}