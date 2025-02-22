// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg1Right extends SequentialCommandGroup {
  /** Creates a new Leg1Right. */
  public Leg1Right(Swerve s_Swerve) {
    addCommands(
      //TODO  add the commands for scoring and receiving coral
 
          //First command to drive with odometry and end 9" from bumper to AprilTag, centered on Tag
          new DriveFwdAndSideAndTurn(s_Swerve, false, 83, -73, -58).withTimeout(1.5),
          
          //Use limelight to get exactly 12" from front frame (9" from bumper) to AprilTag
          new TargetAllParallel(s_Swerve,12, 0).withTimeout(1.5),

          new GetPoseWithLL(s_Swerve),
        
          //Needs to end  with Limelight camera centered 1.6" to the left of the AprilTag center
          new DriveFwdAndSideAndTurn(s_Swerve, false, 10.25, 2.23, 0).withTimeout(1),

          new ResetPoseWithLL(s_Swerve)

    );
  }

}
