// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueRight;

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
    *  alignment for this leg is Right from BLUE driverstation point of view 
    *  robot edge of Bumpers is (18 inches to the LEFT) of center RIGHT side CAGE
    *  this position is aligned with Reef E position as defined in path planner 
   */
  public class BlueRLeg1 extends  SequentialCommandGroup  {
   
  public BlueRLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
    // taken from Path: BlueR_leg1-18-E  (BLUE-left)
    new Pose2d( 7.18, 2.83, new Rotation2d(3.141592653589793) ),
        List.of ( 
            new Translation2d( 6.82871718055536, 2.8379045887056336),
            //new Translation2d( 6.436837125702038, 2.88297329489782),
            new Translation2d( 6.242302142405486, 2.9146377567978705),
            //new Translation2d( 5.850503886012203, 2.9684179287368546),
            new Translation2d( 5.650533161119647, 2.9726512186444083),
            //new Translation2d( 5.251547381789699, 2.8968329538989637),
            new Translation2d( 5.062270707712605, 2.8182024411461866)),
    new Pose2d( 4.884137462852748, 2.7214580978066225, new Rotation2d(2.0943951023931953) ),
    config );

 
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            legTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(legTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
}
