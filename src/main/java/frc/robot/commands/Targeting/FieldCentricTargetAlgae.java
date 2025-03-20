// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.MetricDriveFwdSideTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FieldCentricTargetAlgae extends InstantCommand {
  private Swerve s_Swerve;
  private Pose2d robotFieldPose;
  private Pose2d targetFieldPose;
  private double tv;
  private int targetId;
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  /** Creates a new GoToCoralRightLL. */
  public FieldCentricTargetAlgae(Swerve swerve) {
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
    targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

    if (tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {

      robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
      s_Swerve.setPose(robotFieldPose);

      double x1 = robotFieldPose.getX();
      double y1 = robotFieldPose.getY();
      double angle1 = robotFieldPose.getRotation().getDegrees();

      // april tag coordinates
      double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX();
      double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY();
      double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

      //standoff forward (from testing)
      x2 -= 2.5 * Math.cos((angle2)) * 0.0254;
      y2 -= 2.5 * Math.sin((angle2)) * 0.0254;

      //standoff side (from testing)
      x2 -= 3 * Math.sin((angle2)) * 0.0254;
      y2 += 3 * Math.cos((angle2)) * 0.0254;

      x2 -= Constants.Targeting.DIST_ALGAE_SIDE * Math.sin((angle2)) * 0.0254;
      y2 += Constants.Targeting.DIST_ALGAE_SIDE * Math.cos((angle2)) * 0.0254;

      // double turnAngle = LimelightHelpers.getTargetPose_RobotSpace("limelight")[4];

      // double forward = Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]) - Constants.Targeting.DIST_CAMERA_TO_BUMPER_FWD;
      // double side = Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]);
      
      // side -= Constants.Targeting.DIST_CORAL_TAG_CENTER * Math.cos(Units.degreesToRadians(turnAngle));
      
      // side *= -1;
      // turnAngle *= -1;
      
      SmartDashboard.putNumber("Target ID", targetId);
      SmartDashboard.putNumber("x1: ", x1 / 0.0254);
      SmartDashboard.putNumber("y1: ", y1/ 0.0254);
      SmartDashboard.putNumber("angle1", angle1);
      SmartDashboard.putNumber("x2: ", x2/ 0.0254);
      SmartDashboard.putNumber("y2: ", y2/ 0.0254);
      SmartDashboard.putNumber("angle2", angle2);
     CommandScheduler.getInstance().schedule(new MetricDriveFwdSideTurn(s_Swerve, false, x1, y1, angle1, x2, y2, angle2));
      
    }
  }
}
