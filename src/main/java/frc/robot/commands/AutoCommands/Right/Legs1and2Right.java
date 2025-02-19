// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Legs1and2Right extends SequentialCommandGroup {
  /** Creates a new Legs1and2Right. */
  public Legs1and2Right(Swerve s_Swerve) {
    addCommands(
      new Leg1Right(s_Swerve),
      new Leg2Right(s_Swerve)
    );
  }
}
