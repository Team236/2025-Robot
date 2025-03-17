// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateTargetPosition extends InstantCommand {
  private Swerve s_Swerve;

  public UpdateTargetPosition(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] targetPoseArray = LimelightHelpers.getTargetPose_RobotSpace("limelight"); //TODO: robot space might not be field space, in which case would need to add pose
    SmartDashboard.putNumber("Target Field X (in):", Units.metersToInches(targetPoseArray[0]));
    SmartDashboard.putNumber("Target Field X (m):", targetPoseArray[0]);
    SmartDashboard.putNumber("Target Field Y (in):", Units.metersToInches(targetPoseArray[1]));
    SmartDashboard.putNumber("Target Field Y (m):", targetPoseArray[1]);
    SmartDashboard.putNumber("Target Field Angle:", targetPoseArray[5]);
  }
}
