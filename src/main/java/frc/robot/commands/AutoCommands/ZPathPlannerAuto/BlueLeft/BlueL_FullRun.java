// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft.BlueLLeg1;
import frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft.BlueLLeg2;
import frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft.BlueLLeg3;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueL_FullRun extends SequentialCommandGroup {
  /** Creates a new RRight_FullRun. */
  public BlueL_FullRun(Swerve s_Swerve) {
    addCommands(
      // suggest trying to concatenate the leg3 to leg2 trajectory 
      // this would change to just call one command assuming poses in middle are maintained
      new BlueLLeg1(s_Swerve, false),  //TODO - verify false is correct
      new BlueLLeg2(s_Swerve, false),  //TODO - verify false is correct
      new BlueLLeg3(s_Swerve, false)   //TODO - verify false is correct
    );
  }
}


