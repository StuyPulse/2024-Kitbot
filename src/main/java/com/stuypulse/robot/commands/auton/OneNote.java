package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.drivetrain.DrivetrainRamsete;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneNote extends SequentialCommandGroup {
    // PathPlannerAuto auto = new PathPlannerAuto("One Piece");
    String ONE_PIECE = "One Piece.auto";


    public OneNote(RobotContainer robot) {
        addCommands(
            new DrivetrainRamsete(robot.drivetrain, robot.odometry, ONE_PIECE).robotRelative()
        );
    }
}
