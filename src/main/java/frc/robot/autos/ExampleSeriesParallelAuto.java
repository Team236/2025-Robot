// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ZeroPose;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleSeriesParallelAuto extends SequentialCommandGroup {
  /** Creates a new ExampleSeriesParallelAuto. */
  public ExampleSeriesParallelAuto(Swerve s_Swerve) {
    addCommands(
    Commands.sequence(
    new exampleAuto(s_Swerve, false).withTimeout(3),
    new ZeroPose(s_Swerve).withTimeout(1),
    Commands.parallel(
      new exampleAuto(s_Swerve, false).withTimeout(1),
      new ZeroPose(s_Swerve).withTimeout(1)
      ),
   new exampleAuto(s_Swerve, false) 
    )
    );
  }
}