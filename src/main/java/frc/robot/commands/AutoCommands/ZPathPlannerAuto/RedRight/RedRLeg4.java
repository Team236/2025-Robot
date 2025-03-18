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

  public class RedRLeg4 extends SequentialCommandGroup {
 
  public RedRLeg4(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
         // taken from Path: BlueR_leg4_toC_flipped (RED-RIGHT)
    new Pose2d( 15.94018203751987, 7.354405574562799, new Rotation2d(-2.199114857512855) ),
        List.of ( 
            new Translation2d( 15.717738050840083, 7.083242072698825),
            new Translation2d( 15.46010421115841, 6.778551619455477),
            new Translation2d( 15.329810336005139, 6.627425134994839),
            new Translation2d( 15.067196507707841, 6.326777961430035),
            new Translation2d( 14.935313594493195, 6.176854011291701),
            new Translation2d( 14.671631569304129, 5.876660103754194),
            new Translation2d( 14.540371004677507, 5.725899750381426),
            new Translation2d( 14.280414836361114, 5.421829972318866)),
    new Pose2d( 14.145958342323523, 5.260396207993615, new Rotation2d(-2.0943951023931957) ),
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
