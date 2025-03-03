// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullRunLeftCtr extends SequentialCommandGroup {
  /** Creates a new FullRunLeftCtr. */
  public FullRunLeftCtr(Swerve s_Swerve) {
 
    addCommands(
      

//**** ALSO FIX SO ROBOT USES Limelight TARGETING ON LEG3 (check if needed for Leg1) */
      
      new Leg1LeftCtr(s_Swerve),
      new Leg2LeftCtr(s_Swerve),
      new Leg3LeftCtr(s_Swerve)
    );
  }
}
