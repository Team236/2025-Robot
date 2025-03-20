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
 *  this start position assumes Reef J position as defined in PathPlanner application
*/

public class BlueLLeg2 extends SequentialCommandGroup {

    public BlueLLeg2(Swerve s_Swerve, boolean reversed) {

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // All units in meters.
        Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
                // taken from Path: BlueR_leg2_mirror (BLUE-left)
                new Pose2d(4.884137462852748, 5.330443502193378, new Rotation2d(-2.0943951023931953)),
                List.of(
                        new Translation2d(4.574961007575896, 5.537162106105038),
                        new Translation2d(4.263716396638962, 5.733723800722212),
                        new Translation2d(4.072750027862095, 5.845047362179853)),
                new Pose2d(4.071362051915341, 5.845848715004183, new Rotation2d(-2.0943951023931953)),
                config);

        var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
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
                swerveControllerCommand);
    }
}
