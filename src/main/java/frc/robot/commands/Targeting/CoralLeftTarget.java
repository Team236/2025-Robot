// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralLeftTarget extends SequentialCommandGroup {
  //Targeting with Limelight and then odometry

  public CoralLeftTarget(Swerve s_Swerve) {
    addCommands(  
      new TargetAllParallel(s_Swerve, 10, 0).withTimeout(5.0),
     new GetPoseWithLL(s_Swerve),
     new DriveFwdAndSideAndTurn(s_Swerve, false, 10, 2.23, 0).withTimeout(5),
      new ResetPoseWithLL(s_Swerve));
    
  }

}



