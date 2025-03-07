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
public class CoralRightTarget extends SequentialCommandGroup {
  //Targeting with Limelight and then odometry

  public CoralRightTarget(Swerve s_Swerve) {
    addCommands(
     //new TargetAllParallel(s_Swerve, 0, 0).withTimeout(2),
     new TargetSideDistance(s_Swerve, 0).withTimeout(1),
     new GetPoseWithLL(s_Swerve),
     new DriveFwdAndSideAndTurn(s_Swerve, false, 0, -6.5, 0).withTimeout(1),
     new ResetPoseWithLL(s_Swerve));    
  }

}



