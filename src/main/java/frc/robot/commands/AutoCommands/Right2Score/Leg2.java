// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ZeroPose;
import frc.robot.commands.AutoCommands.TurnDriveFwd;
import frc.robot.commands.AutoCommands.TurnDriveReverse;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveRevAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2 extends SequentialCommandGroup {

    //This is for starting Robot at the Reef after Leg1
  public Leg2(Swerve s_Swerve) {
    addCommands(
    new ZeroPose(s_Swerve).withTimeout(1),
    new DriveReverse(s_Swerve, true, -12).withTimeout(2),
    new TurnOnly(s_Swerve, false,-66.6).withTimeout(2),
    new ZeroPose(s_Swerve).withTimeout(1),
//THIS
    //new DriveRevAndSideAndTurn(s_Swerve, true, -147.5, 57.2, 0).withTimeout(6),
//OR THIS :
  new DriveSideways(s_Swerve, false, 57.2).withTimeout(4),
  new DriveFwd(s_Swerve, true, -147.5).withTimeout(6),

    new ZeroPose(s_Swerve).withTimeout(1)
    );
  }
}
   