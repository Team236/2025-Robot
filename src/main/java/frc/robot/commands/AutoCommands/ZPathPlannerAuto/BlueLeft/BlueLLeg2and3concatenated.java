// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead 
// https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands 
*/
public class BlueLLeg2and3concatenated extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg2and3concatenated(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // leg2 trajectory to follow.  All units in meters.
    Trajectory leg2Trajectory =
    TrajectoryGenerator.generateTrajectory(
       // Start here
        new Pose2d(0, 0, new Rotation2d(0)),
       // Pass through these interior waypoints
       List.of(
        new Translation2d(1,1), 
        new Translation2d(2,2),
        new Translation2d(3,3)
        ),  
        //End here
        new Pose2d(4, 4, new Rotation2d(0)),
        config);

    // leg3 trajectory to add.  All units in meters.
        Trajectory leg3Trajectory =
    TrajectoryGenerator.generateTrajectory(
       // Start here
        new Pose2d(0, 0, new Rotation2d(0)),
       // Pass through these interior waypoints
       List.of(
        new Translation2d(1,1), 
        new Translation2d(2,2),
        new Translation2d(3,3)
        ),  
        //End here
        new Pose2d(4, 4, new Rotation2d(0)),
        config);

    // single trajectory to run as one command 
    // this is assuming that the pose2d between the trajectories is still maintained
    Trajectory combinedTrajectory = leg2Trajectory.concatenate(leg3Trajectory);
 
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            combinedTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(combinedTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
}
