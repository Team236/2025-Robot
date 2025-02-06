// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ZeroPose;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralLeftTarget extends SequentialCommandGroup {
 //This command targets angle and sideways distance in parallel, 
 //then targets forward to 12" from the bumper to the apriltag,
 //then uses odometry to move 12" forward and 6.5" left, to be centered on left coral branch.
  public CoralLeftTarget(Swerve s_Swerve,  double standoffSideways) {
    addCommands(
    new ZeroPose(s_Swerve),
    Commands.parallel(
        new TargetAngle(s_Swerve,  0,0).withTimeout(4),
        new TargetSideDistance(s_Swerve, 0, 0, standoffSideways).withTimeout(4)
      ),
    new TargetForwardDistance(s_Swerve, 0, 0, 15).withTimeout(3),
    new ZeroPose(s_Swerve).withTimeout(1),
    new DriveFwd(s_Swerve, false, 12).withTimeout(3),
    new DriveSideways(s_Swerve, false, 6.5).withTimeout(3)
    );
  }
}
