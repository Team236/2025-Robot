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
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveFwdAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.Scoring.L2_Score;
import frc.robot.commands.Scoring.L4_Score;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.commands.Targeting.TargetAngle;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetSideDistance;
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
      //START ROBOT WITH BACK BUMPER FLUSH WITH BACK OF BLACK STARTING LINE, 91" from sideline
         new DriveFwdAndTurn(s_Swerve, false, 77.5, -58.2).withTimeout(2),
        
          new TargetSideDistance(s_Swerve, 0).withTimeout(1),
          new GetPoseWithLL(s_Swerve).withTimeout(0.25),
          new DriveSideways(s_Swerve, false, -6.25).withTimeout(1.5),
          new DriveFwd(s_Swerve, false, 4.5).withTimeout(2),
          new ResetPoseWithLL(s_Swerve).withTimeout(0.25)

      //If using odometry only, starting 96.25" from side (drive to reef with coral scorer aligned to right branch)
        // new DriveFwdAndSideAndTurn(s_Swerve, false, 78.5, 0, -58.2).withTimeout(3)//,

      , new L2_Score(elevator, coralHold, coralPivot, algaePivot)
    );         
    
  }

}

