// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeTarget extends SequentialCommandGroup {
 //This command targets to be centered on right coral branch.
  public AlgaeTarget(Swerve s_Swerve,  double standoffSideways) {
    addCommands(
   // new TargetSideDistance(s_Swerve, 0).withTimeout(2), //add back if needed
    new TargetAngle(s_Swerve).withTimeout(2),
   // new TargetSideDistance(s_Swerve, 0).withTimeout(1), //add back if needed

   //new DriveFwdAndSideAndTurn(s_Swerve, false, s_Swerve.getLLFwdDistInch(), standoffSideways, 0).withTimeout(2)  //replace below with this if needed
    new DriveFwdAndSideAndTurn(s_Swerve, false, s_Swerve.getLLFwdDistInch(), s_Swerve.getLLSideDistInch(), 0).withTimeout(2)
    ); 
  }
}

