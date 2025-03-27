// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewNewFieldCentricTargetLeft extends SequentialCommandGroup {
  /** Creates a new NewFieldCentricTargetLeft. */
  public NewNewFieldCentricTargetLeft(Swerve s_Swerve) {

    SmartDashboard.putString("Print", "entered NewFieldCentricTargetRight");
    System.out.println("NewFieldCentricTargetRight constructor");
 
      //Get the AprilTag pose from LL 
      double x2 =  s_Swerve.getx2();
      double y2 = s_Swerve.gety2();
      double angle2 = s_Swerve.getAngle2();

      s_Swerve.getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
      
     /*SmartDashboard.putNumber("Target ID", targetId);
      SmartDashboard.putNumber("x1: ", x1 / 0.0254);
      SmartDashboard.putNumber("y1: ", y1/ 0.0254);
      SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
      SmartDashboard.putNumber("x2: ", x2/ 0.0254);
      SmartDashboard.putNumber("y2: ", y2/ 0.0254);
      SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
      */
      double x1 =  s_Swerve.getx1Left();
      double y1 = s_Swerve.gety1Left();
      double angle1 = s_Swerve.getAngle1();

      // DRIVE SEGMENT
       boolean reversed = false; 
       Trajectory exampleTrajectory = s_Swerve.getTargetingTrajectory(x1, y1, angle1, x2, y2, angle2, reversed);
    
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
        
        SmartDashboard.putString("Print", "before addCommands");

        addCommands(
          new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
          new InstantCommand(() -> SmartDashboard.putString("Print", "first instant complete")),
          swerveControllerCommand, 
          new InstantCommand(() -> SmartDashboard.putString("Print", " swerve controller command complete")),
          new ResetFieldPoseWithTarget(s_Swerve),
          new InstantCommand(() -> SmartDashboard.putString("Print", "final: reset field pose complete"))
        );
        SmartDashboard.putString("Print", "after addCommands");
        System.out.println("NewFieldCentricTargetRight after addCommands()");
    //} else {
    //  addCommands(); //TODO  not sure if this is needed? worried code might crash if no target is found ==> no commands are added
    //} 
  }
}

