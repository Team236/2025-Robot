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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BlueLLeg3 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg3(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // Assign trajectory to follow based on allience color.  All units in meters.
    
    if(DriverStation.getAlliance().toString()=="Blue") {  
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d( 1.65695, 0.6915, new Rotation2d(-2.199) ),
        // new Pose2d( 1.6080, 0.69749, new Rotation2d(0.942479) ),
        List.of(
          new Translation2d( 1.830512349159914, 0.9686595273011757),
          new Translation2d( 2.0881461888415886, 1.2733499805445234),
          new Translation2d( 2.2184400639948594, 1.4244764650051616),
          new Translation2d( 2.481053892292155, 1.7251236385699658),
          new Translation2d( 2.612936805506801, 1.8750475887082994),
          new Translation2d( 2.8766188306958678, 2.1752414962458073),
          new Translation2d( 3.0078793953224903, 2.3260018496185744),
          new Translation2d( 3.2678355636388834, 2.6300716276811347) ),
        new Pose2d( 3.4023, 2.7915, new Rotation2d(1.047) ),        // reef C also camera centered
        config );

    } else {
    
    // should be drive leg3 to C or D from from end of leg2 
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d( 1.65695, 0.6915, new Rotation2d(-2.199) ),
        // new Pose2d( 1.6080, 0.69749, new Rotation2d(0.942479) ),
        List.of(
          new Translation2d( 1.830512349159914, 0.9686595273011757),
          new Translation2d( 2.0881461888415886, 1.2733499805445234),
          new Translation2d( 2.2184400639948594, 1.4244764650051616),
          new Translation2d( 2.481053892292155, 1.7251236385699658),
          new Translation2d( 2.612936805506801, 1.8750475887082994),
          new Translation2d( 2.8766188306958678, 2.1752414962458073),
          new Translation2d( 3.0078793953224903, 2.3260018496185744),
          new Translation2d( 3.2678355636388834, 2.6300716276811347) ),
        new Pose2d( 3.4023, 2.7915, new Rotation2d(1.047) ),        // reef C also camera centered
        config );
    }

    // original trajectory generated manually 

/*     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
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
        config); */
 
    var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 
            0, 
            0, 
            Constants.AutoConstants.kThetaControllerConstraints );
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
