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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSideways extends SequentialCommandGroup {
  //Pass in forward X distance (inches, positive), sideways distance (inches), and turn angle (degrees)
  //X always positive, so pass in false for "reversed" in Container when command is called
    public DriveSideways(Swerve s_Swerve, boolean reversed, double sideDist) {
        double sideDistFinal = sideDist * 1.03;
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
             new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these interior waypoints
            List.of(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.1*sideDistFinal) ), 
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.2*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.3*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.4*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.5*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.6*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.7*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.8*sideDistFinal) ),
            new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0.9*sideDistFinal) )),  
     // End here
     new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(sideDistFinal), new Rotation2d(Units.degreesToRadians(0))),
     config);
            
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