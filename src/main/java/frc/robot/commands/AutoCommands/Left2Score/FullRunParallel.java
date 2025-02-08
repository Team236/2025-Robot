// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullRunParallel extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public FullRunParallel(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      //Later add the commands for scoring and receiving coral
      //Later insert actual x y and theta for Leg1 and Leg3

      //LEG 1:
      new DriveFwdAndSideAndTurn(s_Swerve, false, 0, 0, 0),

      //LEG2:
      new DriveFwdAndSideAndTurn(s_Swerve, false, 0, -168, 73),//TESTED on right only


      //LEG3:
      new DriveFwdAndSideAndTurn(s_Swerve, true, 0, 0, 0)

    );
  
  }
}
