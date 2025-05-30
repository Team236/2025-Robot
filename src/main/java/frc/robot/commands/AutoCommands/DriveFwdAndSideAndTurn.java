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

public class DriveFwdAndSideAndTurn extends SequentialCommandGroup {
  //Pass in forward X distance (inches, positive), sideways distance (inches), and turn angle (degrees)
  //X (fwdDist) always positive, so pass in false for "reversed" in Container when command is called
    //***** X (fwdDist) MUST BE POSITIVE! 
  //Pass in false for "reversed" in Container when command is called
    public DriveFwdAndSideAndTurn(Swerve s_Swerve, boolean reversed, double fwdDist, double sideD, double turnAngle) {
        
        //****WAS NOT DRIVINg ENOUGH SIDEWAYS WITHOUT THIS FACTOR!!*****
        double sideDist = sideD * 1.00; //1.03 //TODO find this factor for 2025 


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
            List.of(
                   new Translation2d(Units.inchesToMeters(0.05*fwdDist), Units.inchesToMeters(0.05*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.1*fwdDist), Units.inchesToMeters(0.1*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.15*fwdDist), Units.inchesToMeters(0.15*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.2*fwdDist), Units.inchesToMeters(0.2*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.25*fwdDist), Units.inchesToMeters(0.25*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.3*fwdDist), Units.inchesToMeters(0.3*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.35*fwdDist), Units.inchesToMeters(0.35*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.4*fwdDist), Units.inchesToMeters(0.4*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.45*fwdDist), Units.inchesToMeters(0.45*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.5*fwdDist), Units.inchesToMeters(0.5*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.55*fwdDist), Units.inchesToMeters(0.55*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.6*fwdDist), Units.inchesToMeters(0.6*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.65*fwdDist), Units.inchesToMeters(0.65*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.7*fwdDist), Units.inchesToMeters(0.7*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.75*fwdDist), Units.inchesToMeters(0.75*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.8*fwdDist), Units.inchesToMeters(0.8*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.85*fwdDist), Units.inchesToMeters(0.85*sideDist)), 
                   new Translation2d(Units.inchesToMeters(0.9*fwdDist), Units.inchesToMeters(0.9*sideDist)),
                   new Translation2d(Units.inchesToMeters(0.95*fwdDist), Units.inchesToMeters(0.95*sideDist))
                   ),  
            // End here
            new Pose2d(Units.inchesToMeters(fwdDist), Units.inchesToMeters(sideDist), new Rotation2d(Units.degreesToRadians(turnAngle))),
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