/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDriveForever;
import com.stuypulse.robot.commands.launcher.LaunchPrepare;
import com.stuypulse.robot.commands.launcher.LauncherLaunch;
import com.stuypulse.robot.commands.launcher.LauncherIntakeNote;
import com.stuypulse.robot.commands.launcher.LauncherStop;
import com.stuypulse.robot.commands.odometry.OdometryRealign;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.launcher.Launcher;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;
import com.stuypulse.robot.subsystems.vision.AbstractVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AbstractDrivetrain drivetrain = AbstractDrivetrain.getInstance();
    public final Launcher launcher = Launcher.getInstance();
    //public final AbstractOdometry odometry = AbstractOdometry.getInstance();
    //public final AbstractVision vision = AbstractVision.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        //drivetrain.setDefaultCommand(new DrivetrainDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        driver.getRightBumper()
            .whileTrue(new LauncherIntakeNote())
            .onFalse(new LauncherStop());
    
        driver.getBottomButton()
            .whileTrue(new WaitCommand(0.5).andThen(new LauncherLaunch()))
            .onFalse(new LauncherStop());
    }

    private void configureOperatorBindings() {
        driver.getLeftButton().onTrue(new DrivetrainDriveForever(2));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
