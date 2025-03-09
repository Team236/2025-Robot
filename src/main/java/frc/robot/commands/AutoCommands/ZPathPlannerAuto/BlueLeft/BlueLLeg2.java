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
public class BlueLLeg2 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg2(Swerve s_Swerve, boolean reversed) {
    
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // ***** Path: leg2***** 
    Trajectory exampleTrajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d( 5.159896066581073, 2.889722580682677, new Rotation2d(2.0943951023931953) ),
        List.of( 
            new Translation2d( 4.828341742659023, 2.680378129716683),
            new Translation2d( 4.487826735872669, 2.461020827766689),
            new Translation2d( 4.3147710420532945, 2.3484878281679777),
            new Translation2d( 3.964187404857987, 2.119514072713695),
            new Translation2d( 3.787109114000389, 2.003793693254352),
            new Translation2d( 3.4305037190611367, 1.7716868718618328),
            new Translation2d( 3.251426267497817, 1.6560208063248845),
            new Translation2d( 2.8928459874796264, 1.4272643065601804),
            new Translation2d( 2.7137928115430894, 1.314894248728653),
            new Translation2d( 2.3572845191109675, 1.0959714581578175),
            new Translation2d( 2.1802790551337163, 0.9901391018147371),
            new Translation2d( 1.82988962295267, 0.7875334080038218) ),
        new Pose2d( 1.65695, 0.6915, new Rotation2d(-2.199) ),
        config );

    // An example trajectory to follow.  All units in meters.
    // original path data 
    /* Trajectory exampleTrajectory =  TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0) ),    // Start here
       List.of(                                 // Pass through these interior waypoints
        new Translation2d(1,1), 
        new Translation2d(2,2),
        new Translation2d(3,3) ),
        new Pose2d(4, 4, new Rotation2d(0) ),    //End here
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
