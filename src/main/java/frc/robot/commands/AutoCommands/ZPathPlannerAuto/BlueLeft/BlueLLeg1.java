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
   https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

   /* 
    *  alignment for this leg is LEFT from BLUE driverstation point of view 
    *  LEFT uses flip method to generate from PathPlanner 
    *  robot edge of Bumpers is (18 inches to the RIGHT) of center LEFT side CAGE
    *  this position is aligned with Reef J position as defined in path planner 
   */
public class BlueLLeg1 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // from BlueLL-leg1E-18-MIRROR.txt positions  All units in meters.
        Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
           // Start here
            new Pose2d( 12.388354333418924, 5.162179019317324, new Rotation2d(-1.0471975511965979) ),
                   // Pass through these interior waypoints
            List.of ( 
              new Translation2d( 12.719908657340973, 5.371523470283318),
              //new Translation2d( 13.060423664127327, 5.590880772233312),
              new Translation2d( 13.233479357946702, 5.7034137718320235),
              // new Translation2d( 13.584062995142009, 5.932387527286306),
              new Translation2d( 13.761141285999608, 6.048107906745649),
              //new Translation2d( 14.117746680938861, 6.2802147281381675),
              new Translation2d( 14.29682413250218, 6.395880793675117),
              // new Translation2d( 14.65540441252037, 6.62463729343982),
              new Translation2d( 14.834457588456907, 6.737007351271348),
              // new Translation2d( 15.19096588088903, 6.955930141842183),
              new Translation2d( 15.36797134486628, 7.061762498185264),
              new Translation2d( 15.718360777047327, 7.264368191996179) ),
            new Pose2d( 15.891295092732786, 7.360421153067785, new Rotation2d(0.9424777960769383) ),
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
