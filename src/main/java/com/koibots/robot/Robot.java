// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.koibots.robot;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;



/**
 * The VM is configured to automatically run this class. If you change the name of this class or the
 * package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
    VictorSPX motor1;
    VictorSPX motor2;
    VictorSPXConfiguration motorConfig = new VictorSPXConfiguration();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table;

    final double G_fps = 32.1741;
    final double theta_rad = StrictMath.PI / 4;
    public void robotInit() {
        motor1 = new VictorSPX(0);
        motor2 = new VictorSPX(1);

        motor2.follow(motor1);

        table = nt.getTable("Inputs");
    }
    

    public void disabled() {
        motor1.set(VictorSPXControlMode.PercentOutput, 0);
    }


    public void autonomous() {
        double height = table.getEntry("Height").getDouble(0);
        double targetDistance = table.getEntry("Distance").getDouble(4);

        double flywheelVelocity = StrictMath.sqrt((G_fps * targetDistance) / (StrictMath.sin(2 * theta_rad)));

        motor1.set(VictorSPXControlMode.Velocity, flywheelVelocity);
    }
}
