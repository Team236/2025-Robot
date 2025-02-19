// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class LLFullRunRight extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public LLFullRunRight(Swerve s_Swerve) {
    addCommands(
      //TODO  add the commands for scoring and receiving coral

          //LEG 1:  
          //First command to drive with odometry and end 9" from bumper to AprilTag, centered on Tag
          new DriveFwdAndSideAndTurn(s_Swerve, false, 63.6, -85.35, -58).withTimeout(3),
          //Use limelight to get exactly 12" from front frame (9" from bumper) to AprilTag
          new TargetAllParallel(s_Swerve,12, 0).withTimeout(2),
          //Needs to end  with Limelight camera centered 1.6" to the left of the AprilTag center
          new DriveFwdAndSideAndTurn(s_Swerve, false, 10, 1.6, 0),
        
          //LEG2:
          //Tweak as needed to end against the Coral Loading station, not close to side of field
          //note - camera on 2025 robot will be on opposite side of robot compared with the swerve testbed
          new DriveSideways(s_Swerve, false, 84).withTimeout(2),
          new DriveFwdAndSideAndTurn(s_Swerve, false, -22, 90, -68),

          //LEG3:
          //First command to drive with odometry and end 9" from bumper to AprilTag, centered on Tag  
          new DriveFwdAndSideAndTurn(s_Swerve, true,120, -8, 6).withTimeout(3.5),
          //Use limelight to get exactly 12" from front frame (9" from bumper) to AprilTag
          new TargetAllParallel(s_Swerve, 12, 0).withTimeout(2),
          //Needs to end  with Limelight camera centered 0.4" to the left of the AprilTag center
          new DriveFwdAndSideAndTurn(s_Swerve, false, 9, 1.6, 0)

    );
  
  }
}