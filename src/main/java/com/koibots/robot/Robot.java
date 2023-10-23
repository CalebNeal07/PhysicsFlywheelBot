/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.koibots.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
    private Joystick m_stick;
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    @Override
    public void robotInit() {
        m_stick = new Joystick(0);

        // initialize motor
        motor1 = new CANSparkMax(1, MotorType.kBrushed);
        motor2 = new CANSparkMax(2, MotorType.kBrushed);

        motor2.follow(motor1, false);


        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        motor1.restoreFactoryDefaults();

        /**
         * In order to use PID functionality for a controller, a SparkMaxPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        pidController = motor1.getPIDController();

        // Encoder object created to display position values
        encoder = motor1.getEncoder();

        // PID coefficients
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 4200;

        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    @Override
    public void teleopPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
        }

        /**
         * PIDController objects are commanded to a set point using the
         * SetReference() method.
         *
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         *
         * The second parameter is the control type can be set to one of four
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        double setPoint = m_stick.getY()*maxRPM;
        pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());
    }
}