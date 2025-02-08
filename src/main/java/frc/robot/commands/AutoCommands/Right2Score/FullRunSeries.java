// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right2Score;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ZeroPose;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullRunSeries extends SequentialCommandGroup {
  /** Creates a new FullRun. */
  public FullRunSeries(Swerve s_Swerve) {
    addCommands(
    new Leg1Series(s_Swerve).withTimeout(10),
    new ZeroPose(s_Swerve).withTimeout(1),
    new Leg2Series(s_Swerve).withTimeout(10),
    new ZeroPose(s_Swerve).withTimeout(1),
    new Leg3Series(s_Swerve).withTimeout(10),
    new ZeroPose(s_Swerve).withTimeout(1)
    );
  }
}
   