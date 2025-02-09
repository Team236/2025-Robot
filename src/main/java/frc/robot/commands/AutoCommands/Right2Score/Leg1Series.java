// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg1Series extends SequentialCommandGroup {

    //This is for starting Robot on right side of field (wrt DS), 12" from the side,
    //with bumpers parallel to and just over the starting line
  public Leg1Series(Swerve s_Swerve) {
    
    addCommands(
    new DriveFwd(s_Swerve, false, 26).withTimeout(2),
    new TurnOnly(s_Swerve, false, -60).withTimeout(2),
   // new TurnDriveFwd(s_Swerve, false, -60, 100).withTimeout(3),
    new DriveFwd(s_Swerve, false, 100).withTimeout(3)
    );
  }
}
   