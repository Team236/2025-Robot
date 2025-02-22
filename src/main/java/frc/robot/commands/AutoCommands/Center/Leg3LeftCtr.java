// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.Targeting.CoralRightTarget;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg3LeftCtr extends SequentialCommandGroup {
  /** Creates a new Leg3LeftCtr. */
  public Leg3LeftCtr(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveFwdAndSideAndTurn(s_Swerve, true,
      Constants.AutoConstants.LEFT_CENTER_LEG3_FWD_X, 
      Constants.AutoConstants.LEFT_CENTER_LEG3_SIDE_Y,
      Constants.AutoConstants.LEFT_CENTER_LEG3_ANGLE),
      new CoralRightTarget(s_Swerve)
    );
  }
}
