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
          new DriveFwdAndSideAndTurn(s_Swerve, false, 63.6, -85.35, -58).withTimeout(3),
          new TargetAllParallel(s_Swerve,12, 0).withTimeout(2),
          new DriveFwdAndSideAndTurn(s_Swerve, false, 9, -6, 0),
        
          //LEG2:
          new DriveSideways(s_Swerve, false, 84).withTimeout(2),
          new DriveFwdAndSideAndTurn(s_Swerve, false, -15, 97, -68),
          //LEG3:
          new DriveFwdAndSideAndTurn(s_Swerve, true,120, -13, 6).withTimeout(3.5),
          new TargetAllParallel(s_Swerve, 12, 0).withTimeout(2)

    );
  
  }
}