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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RedLLeg1 extends  SequentialCommandGroup  {
   
  public RedLLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
     // taken from Path: BlueR_leg1-18-E_mirror_flip  (RED-left)
     new Pose2d( 10.368250399999997, 2.83, new Rotation2d(-0.0) ),
          List.of ( 
               new Translation2d( 10.719533219444637, 2.8379045887056336),
               new Translation2d( 11.11141327429796, 2.8829732948978197),
               new Translation2d( 11.305948257594512, 2.9146377567978705),
               new Translation2d( 11.697746513987795, 2.968417928736855),
               new Translation2d( 11.89771723888035, 2.9726512186444083),
               new Translation2d( 12.296703018210298, 2.8968329538989632),
               new Translation2d( 12.485979692287392, 2.8182024411461866)),
     new Pose2d( 12.664112937147248, 2.7214580978066225, new Rotation2d(1.0471975511965979) ),
     config );
 
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
