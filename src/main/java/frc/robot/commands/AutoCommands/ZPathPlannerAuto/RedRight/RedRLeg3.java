// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.RedRight;

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
    *  alignment for this leg is RIGHT from RED driverstation point of view 
    * 
    *  this start position assumes last leg drove away from 
    *  RED REEF E position as defined in PathPlanner application
    */

  public class RedRLeg3 extends SequentialCommandGroup {
 
  public RedRLeg3(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
        // taken from Path: BlueR_leg3_flipped (RED-RIGHT)
        new Pose2d( 13.476888348084657, 5.845848715004183, new Rotation2d(-1.0471975511965979) ),
            List.of ( 
                new Translation2d( 13.743881932958502, 6.012916277306253),
                new Translation2d( 14.055697097808213, 6.170505775620173),
                new Translation2d( 14.23837537743004, 6.25023160028466),
                new Translation2d( 14.608206988775365, 6.4002194593694846),
                new Translation2d( 14.792584726951727, 6.4772872967760176),
                new Translation2d( 15.152098351657774, 6.653585082198218),
                new Translation2d( 15.323176644698574, 6.7587647500072014),
                new Translation2d( 15.599780254630128, 6.974784667758368),
                new Translation2d( 15.71213834414275, 7.083752438560468)),
        new Pose2d( 15.94018203751987, 7.354405574562799, new Rotation2d(-2.199114857512855) ),
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
