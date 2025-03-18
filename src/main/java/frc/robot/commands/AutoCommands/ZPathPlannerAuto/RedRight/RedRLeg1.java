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
import edu.wpi.first.math.util.Units;
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
  public class RedRLeg1 extends  SequentialCommandGroup  {
  
  public RedRLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    //  All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
        // taken from Path: BlueR_leg1-18-E_flipped  (RED-RIGHT)
        new Pose2d( 10.368250399999997, 5.221901600000001, new Rotation2d(0.0) ),
            List.of ( 
                new Translation2d( 10.719533219444637, 5.213997011294367),
                new Translation2d( 11.11141327429796, 5.168928305102181),
                new Translation2d( 11.305948257594512, 5.13726384320213),
                new Translation2d( 11.697746513987795, 5.083483671263146),
                new Translation2d( 11.89771723888035, 5.0792503813555925),
                new Translation2d( 12.296703018210298, 5.1550686461010375),
                new Translation2d( 12.485979692287392, 5.233699158853814)),
        new Pose2d( 12.664112937147248, 5.330443502193378, new Rotation2d(-1.0471975511965979) ),
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
