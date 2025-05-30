// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeTarget extends SequentialCommandGroup {
  //Targeting with Limelight and then odometry

  public AlgaeTarget(Swerve s_Swerve) {
    addCommands(
//TODO: Try the 3 commands below vice the TargetSideDist only
//If fast enough, this is more likely to get us properly aligned
//Adjust timeouts to save time
    //new TargetAngle(s_Swerve).withTimeout(0.7),
    //new TargetSideDistance(s_Swerve, 0).withTimeout(1),
    //new TargetForwardDistance(s_Swerve, 0.7),
    
      new TargetSideDistance(s_Swerve, 0).withTimeout(1),
      new GetPoseWithLL(s_Swerve).withTimeout(0.5),
      //TODO - get actual values for side distance algae
      new DriveFwdAndSideAndTurn(s_Swerve, false, 1, -9, 0).withTimeout(2),
      new ResetPoseWithLL(s_Swerve).withTimeout(0.5));
  }

}
