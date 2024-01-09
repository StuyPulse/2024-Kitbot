package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LaunchNote extends InstantCommand {
    Launcher launcher;

    public LaunchNote() {
        launcher = Launcher.getInstance();
        addRequirements(launcher);
    }
    
    @Override
    public void initialize() {
        launcher.launch();
    }
}