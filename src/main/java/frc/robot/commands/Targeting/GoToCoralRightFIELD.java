// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class GoToCoralRightFIELD extends SequentialCommandGroup {


    public GoToCoralRightFIELD(Swerve s_Swerve) {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
        
        Optional<Alliance> alliance = DriverStation.getAlliance(); 
        
        if (alliance.isPresent() && tv == 1) {

          int targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

          if (Constants.Targeting.REEF_IDS.contains(targetId)) {
            
            // getting the robots initial pose
            Pose2d robotFieldPose = new Pose2d();
            if (alliance.get() == Alliance.Red) {
              robotFieldPose = LimelightHelpers.getBotPose2d_wpiRed("limelight");
              s_Swerve.setPose(robotFieldPose);
            }
            else if (alliance.get() == Alliance.Blue) {
              robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
              s_Swerve.setPose(robotFieldPose);
            }
  
            // getting the target pose
            Pose2d targetFieldPose = Constants.Targeting.ID_TO_POSE.get(targetId);
            // Pose2d targetFieldPose = LimelightHelpers.getTargetPose_RobotSpace("limelight");

            // calculating angle offset between robot and target so that we can turn initially?
            //TODO: sequential command group so will increase time spent doing this command?
            double targetAngle = targetFieldPose.getRotation().getDegrees();
            double turnAngle = targetAngle - robotFieldPose.getRotation().getDegrees(); //TODO: may have to invert direction of this angle

            // calculating the end pose (which includes offset)
            // TODO: may have to swap around these trig functions
            Pose2d endFieldPose = targetFieldPose.plus(new Transform2d(Constants.Targeting.DIST_CORAL_TAG_CENTER * Math.sin(Units.degreesToRadians(targetAngle)), Constants.Targeting.DIST_CORAL_TAG_CENTER * Math.cos(Units.degreesToRadians(targetAngle)), new Rotation2d()));
    
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics).setReversed(false);
    
            List<Translation2d> waypoints = new ArrayList<>();
            double interval = 1.0 / (Constants.Targeting.TRAJECTORY_NUM_WAYPOINTS + 1);
            for (int i = 1; i <= Constants.Targeting.TRAJECTORY_NUM_WAYPOINTS; i++) {
                double x = Units.inchesToMeters(robotFieldPose.getX() + i * interval * (targetFieldPose.getX() - robotFieldPose.getX()));
                double y = Units.inchesToMeters(robotFieldPose.getY() + i * interval * (targetFieldPose.getY() - robotFieldPose.getY()));
                waypoints.add(new Translation2d(x, y));
            } 
    
            // An example trajectory to follow.  All units in meters.
            Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at robot field pose
                 robotFieldPose,
                // Pass through these interior waypoints
                waypoints,
                // End at the target field pose + offset
                endFieldPose,
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
                new TurnOnly(s_Swerve, false, turnAngle),
                new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
                swerveControllerCommand
            );
          }

        }
    }
}
