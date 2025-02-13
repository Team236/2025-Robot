// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeTarget extends SequentialCommandGroup {
  /** Creates a new TargetAllSeries. */  
  //Targeting with Limelight
  public AlgaeTarget(Swerve s_Swerve) {
    addCommands(
       //TODO:  TRY TargetAllParallel with 0 standoffSideways, and then DriveFwdAndSide 
      new TargetAllParallel(s_Swerve, 9, 6.5).withTimeout(1.5),
      new DriveFwd(s_Swerve, false, 9).withTimeout(1).withTimeout(3));

    //  new TargetAllParallel(s_Swerve, 9, 0).withTimeout(1.5),
    // new DriveFwdAndSideAndTurn(s_Swerve, false, 9, 6.5, 0).withTimeout(3);
}
}
