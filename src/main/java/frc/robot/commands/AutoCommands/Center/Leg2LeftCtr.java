// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2LeftCtr extends SequentialCommandGroup {
  /** Creates a new Leg2LeftCtr. */
  public Leg2LeftCtr(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveReverse(s_Swerve, true, Constants.AutoConstants.LEFT_CENTER_LEG2_REVERSE_DIST).withTimeout(2),
      new DriveFwdAndSideAndTurn(s_Swerve, false,
      Constants.AutoConstants.LEFT_CENTER_LEG2_A_FWD_X, 
      Constants.AutoConstants.LEFT_CENTER_LEG2_A_SIDE_Y, 
      Constants.AutoConstants.LEFT_CENTER_LEG2_A_ANGLE),
      new DriveFwdAndSideAndTurn(s_Swerve, false,
      Constants.AutoConstants.LEFT_CENTER_LEG2_B_FWD_X, 
      Constants.AutoConstants.LEFT_CENTER_LEG2_B_SIDE_Y, 
      Constants.AutoConstants.LEFT_CENTER_LEG2_B_ANGLE_CCW)
    );
  }
}
