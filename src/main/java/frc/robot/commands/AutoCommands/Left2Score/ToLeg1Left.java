// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ToLeg1Left extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public ToLeg1Left(Swerve s_Swerve) {
    addCommands(
      //TODO  add the commands for scoring and receiving coral

      //LEG 1:
      new DriveFwdAndSideAndTurn(s_Swerve, false, 
      Constants.AutoConstants.LEFT_LEG1_FWD_X, 
      Constants.AutoConstants.LEFT_LEG1_SIDE_Y, 
      Constants.AutoConstants.LEFT_LEG1_ANGLE_CCW),
    );
  
  }
}