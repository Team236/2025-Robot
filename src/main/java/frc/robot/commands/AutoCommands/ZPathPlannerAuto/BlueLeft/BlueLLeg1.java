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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlueLLeg1 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // An example trajectory to follow.  All units in meters.

            
        // replace this with leg?? pose and waypoints
        //TODO where does this drive replace this with leg??  pose and waypoints 


    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //***** Path: leg1.txt ***** 
        new Pose2d( 7.17, 2.83, new Rotation2d(3.141592653589793) ),
        List.of ( 
            new Translation2d( 6.760171794428205, 2.771238870019034),
            new Translation2d( 6.357486066409191, 2.7219606838198507),
            new Translation2d( 6.157303328740488, 2.6980566302568914),
            new Translation2d( 5.758469081546904, 2.648993297562662),
            new Translation2d( 5.559727960470591, 2.6230216173273404),
            new Translation2d( 5.163674217701479, 2.5663090837843123) ),
        new Pose2d( 4.958352911410241, 2.5333954076599787, new Rotation2d(2.08265216922672) ),
        config );
        // original path data 
    /* Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d( 0, 0, new Rotation2d(0) ),   // Start here
       List.of(                                 // Pass through these interior waypoints
        new Translation2d( 1, 1), 
        new Translation2d( 2, 2),
        new Translation2d( 3, 3) ),  
        new Pose2d(4, 4, new Rotation2d(0) ),   //End here
        config ); 
    */
 
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
}
