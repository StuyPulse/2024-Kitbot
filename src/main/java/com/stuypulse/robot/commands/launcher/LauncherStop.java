package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LauncherStop extends InstantCommand{
    Launcher launcher;

    public LauncherStop() {
        launcher = Launcher.getInstance();
        addRequirements(launcher);
    }
    
    @Override
    public void initialize() {
        launcher.stop();
    }
    
}
