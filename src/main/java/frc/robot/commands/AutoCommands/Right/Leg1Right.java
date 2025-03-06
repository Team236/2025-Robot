// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ZeroPose;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.DangerPIDToHeight;
import frc.robot.commands.Scoring.L4_Score;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg1Right extends SequentialCommandGroup {
  /** Creates a new Leg1Right. */
  public Leg1Right(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
      //START ROBOT WITH BACK BUMPER FLUSH WITH BACK OF BLACK STARTING LINE
          Commands.parallel(
          //If using odometry plus LL:
          //First command to drive with odometry and end ~7" from bumper to AprilTag, centered on Tag
          //new DriveFwdAndSideAndTurn(s_Swerve, false, 82, 0, -58.2)//,//.withTimeout(3),

          //If using odometry only, starting 96.25" from side (drive to reef with coral scorer aligned to right branch)
            new DriveFwdAndSideAndTurn(s_Swerve, false, 77.5, 0, -58.2)//,//.withTimeout(3),
            
          //new PIDToElevSafePosition(algaePivot)
             )//,

          //If using odometry plus LL:
          //Use limelight to get exactly 7" from bumper to AprilTag
          //new TargetAllParallel(s_Swerve, 7, 0)//,//.withTimeout(1),
          // new GetPoseWithLL(s_Swerve),
          // new DriveFwdAndSideAndTurn(s_Swerve, false, 7, 2.23, 0),//.withTimeout(1),
          // new ResetPoseWithLL(s_Swerve)//,

          //new PIDToElevSafePosition(algaePivot),
          //new L4_Score(elevator, coralHold, coralPivot, algaePivot)
          
    );
    
  }

}

