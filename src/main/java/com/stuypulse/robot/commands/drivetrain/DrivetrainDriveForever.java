/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainDriveForever extends Command {
    private final AbstractDrivetrain drivetrain;
    private final double speed;

    public DrivetrainDriveForever(double speed) {
        this.drivetrain = AbstractDrivetrain.getInstance();
        this.speed = speed;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.tankDriveVolts(speed, speed);
    }
}
