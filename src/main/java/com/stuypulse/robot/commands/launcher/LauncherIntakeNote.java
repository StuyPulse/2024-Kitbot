package com.stuypulse.robot.commands.launcher;

import com.stuypulse.robot.subsystems.launcher.Launcher;

import edu.wpi.first.wpilibj2.command.Command;

public class LauncherIntakeNote extends Command {
    Launcher launcher;

    public LauncherIntakeNote() {
        launcher = Launcher.getInstance();
        addRequirements(launcher);
    }
    
    @Override
    public void execute() {
        launcher.intake();
    } 
}
