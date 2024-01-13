package com.stuypulse.robot.commands.drivetrain;

import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.stuypulse.robot.subsystems.drivetrain.AbstractDrivetrain;
import com.stuypulse.robot.subsystems.odometry.AbstractOdometry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DrivetrainPPRamsete extends FollowPathRamsete  /*Find the PP Drive Path Command*/ {
    private boolean resetPosition;
    private AbstractOdometry odometry;
    private PathPlannerTrajectory trajectory;

    public DrivetrainPPRamsete(AbstractDrivetrain drivetrain, AbstractOdometry odometry, PathPlannerPath path){
        super(
            path, 
            odometry::getPose, 
            drivetrain::getChassisSpeeds,
            drivetrain::chassisSpeedsDrive, 
            new ReplanningConfig(),
            shouldFlipPath(),
            drivetrain
        );

        this.resetPosition = true;
        this.odometry = odometry;
        this.trajectory = path.getTrajectory(null, null);
    }


    //TODO: Decide which side we are flipping the path on 
    private static BooleanSupplier shouldFlipPath() {
        return () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == Alliance.Red;
            } else {
                return false;
            }
        };
    }

    public DrivetrainPPRamsete robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainPPRamsete fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        super.initialize();

        if (resetPosition) {
            this.odometry.resetOdometery(trajectory.getInitialDifferentialPose());
        }
    }


}


	
/*
     get the drivetrain 
     get the odometry
     get the trajectories, seperate the paths by strin
     boolean for robotRelatives, shouldStop
     Hashmap<String, Command> of events

        methods
        addEvents 
        withEvents

        initalize shoudl reseeverything 

        end on if shouldStop true and then stop the robot and then set the pose to 0, 0
s
    */
