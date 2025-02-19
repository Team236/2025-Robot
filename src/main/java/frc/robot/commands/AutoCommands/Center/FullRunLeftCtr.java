// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullRunLeftCtr extends SequentialCommandGroup {
  /** Creates a new FullRunLeftCtr. */
  public FullRunLeftCtr(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //LEG 1
      new DriveFwd(s_Swerve, false, Constants.AutoConstants.LEFT_CENTER_LEG1_FWD_DIST),

      //LEG 2
      new DriveReverse(s_Swerve, true, Constants.AutoConstants.LEFT_CENTER_LEG2_REVERSE_DIST).withTimeout(2),
      new DriveFwdAndSideAndTurn(s_Swerve, false,
      /*Constants.AutoConstants.LEFT_CENTER_LEG2_FWD_X */0, 
      /*Constants.AutoConstants.LEFT_CENTER_LEG2_SIDE_Y*/45, 
      /*Constants.AutoConstants.LEFT_CENTER_LEG2_ANGLE_CCW*/0),


      //LEG 3
      new DriveFwdAndSideAndTurn(s_Swerve, false, 190, 46, 50)
      
      //LEG 3
    );
  }
}
