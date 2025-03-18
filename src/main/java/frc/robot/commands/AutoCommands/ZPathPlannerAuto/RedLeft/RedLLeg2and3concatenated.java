// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.RedLeft;

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
   https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

   /* 
    *  alignment for this leg is LEFT from RED driverstation point of view 
    *  this start position assumes Reef J position as defined in PathPlanner application
   */
  public class RedLLeg2and3concatenated extends  SequentialCommandGroup  {
  
  public RedLLeg2and3concatenated(Swerve s_Swerve, boolean reversed) {
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // All units in meters.
    Trajectory leg2Trajectory = TrajectoryGenerator.generateTrajectory(
        // taken from Path: BlueR_leg2_mirror_flip (RED-left)
        new Pose2d( 12.664112937147248, 2.7214580978066225, new Rotation2d(1.0471975511965979) ),
            List.of ( 
                new Translation2d( 12.9732893924241, 2.5147394938949628),
                new Translation2d( 13.284534003361035, 2.3181777992777883),
                new Translation2d( 13.475500372137901, 2.2068542378201474)),
        new Pose2d( 13.476888348084657, 2.2060528849958176, new Rotation2d(1.0471975511965979) ),
        config );

    // All units in meters.
    Trajectory leg3Trajectory = TrajectoryGenerator.generateTrajectory(
        // taken from Path: BlueR_leg3_mirror_flip  (RED-left)
        new Pose2d( 13.476888348084657, 2.2060528849958176, new Rotation2d(1.0471975511965979) ),
            List.of ( 
                new Translation2d( 13.743881932958502, 2.0389853226937475),
                //new Translation2d( 14.055697097808213, 1.8813958243798279),
                new Translation2d( 14.23837537743004, 1.8016699997153411),
                //new Translation2d( 14.608206988775365, 1.6516821406305162),
                new Translation2d( 14.792584726951727, 1.5746143032239832),
                //new Translation2d( 15.152098351657774, 1.3983165178017831),
                new Translation2d( 15.323176644698574, 1.2931368499927993),
                //new Translation2d( 15.599780254630128, 1.0771169322416325),
                new Translation2d( 15.71213834414275, 0.9681491614395323)),
            new Pose2d( 15.94018203751987, 0.6974960254372018, new Rotation2d(2.199114857512855) ),
            config );

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
