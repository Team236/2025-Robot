package frc.robot.commands.AutoCommands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveWithPath extends SequentialCommandGroup {
  //Pass in forward X distance (inches, positive), sideways distance (inches), and turn angle (degrees)
  //X (fwdDist) always positive, so pass in false for "reversed" in Container when command is called


    public DriveWithPath(Swerve s_Swerve, boolean reversed) {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(7.170291487008191,7.170291487008191,new Rotation2d(-3.1396414314172425)),
	// Pass through these interior waypoints
	List.of(
		new Translation2d(6.270045790650829,1.6295163257959588),
		new Translation2d(6.107231772927737,1.8028919298664268),
		new Translation2d(5.945051892036425,1.977142632681524) ,
		new Translation2d(5.787114115105524,2.147955465660495) ,
		new Translation2d(5.637026409263667,2.3110174602225864),
		new Translation2d(5.498396741639484,2.462015647787042) ,
		new Translation2d(5.374833079361604,2.5966370597731077),
		new Translation2d(5.2699433895586605,2.7105687276000285),
		new Translation2d(5.200108901567801,2.785839613470134) ,
		new Translation2d(5.14741654092824,2.841727318992451)  ,
		new Translation2d(5.117537960654374,2.8621667684629086),
		new Translation2d(5.117537960654374,2.8621667684629086)),
	new Pose2d(5.117537960654374,5.117537960654374,new Rotation2d(2.096022658820897)),config);
            
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