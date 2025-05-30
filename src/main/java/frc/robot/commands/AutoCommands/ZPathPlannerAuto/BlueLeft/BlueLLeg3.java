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
    * 
    *  this start position assumes last leg drove away from 
    *  Reef E position as defined in PathPlanner application
   */
public class BlueLLeg3 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueLLeg3(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // values taken from Leg2b-FLIPPED.txt positions  All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
       // previous leg2a drove away from Corral E
       new Pose2d( 4.071362051915341, 2.2060528849958176, new Rotation2d(2.0943951023931953) ),
       // Pass through these interior waypoints
        List.of ( 
          new Translation2d( 3.8043684670414954, 2.038985322693748),
          //new Translation2d( 3.4925533021917827, 1.8813958243798283),
          new Translation2d( 3.309875022569957, 1.8016699997153411),
          //new Translation2d( 2.9400434112246323, 1.651682140630516),
          new Translation2d( 2.75566567304827, 1.5746143032239828),
          //new Translation2d( 2.396152048342224, 1.3983165178017831),
          new Translation2d( 2.225073755301423, 1.2931368499927995),
          //new Translation2d( 1.9484701453698698, 1.0771169322416327),
          new Translation2d( 1.8361120558572477, 0.9681491614395328)),
        new Pose2d( 1.6080683624801264, 0.6974960254372014, new Rotation2d(0.9424777960769379) ),
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
