// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.Scoring.L2_Score;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class CtrScore1 extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public CtrScore1(Swerve s_Swerve, Elevator elevator, AlgaePivot algeaPivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
      //TODO:  add the commands for scoring 
      // Commands.parallel(
        new DriveFwd(s_Swerve, false, Constants.AutoConstants.CENTER_FWD_DIST).withTimeout(2),
        new TargetSideDistance(s_Swerve, 0).withTimeout(1),
        new GetPoseWithLL(s_Swerve).withTimeout(0.25),
        new DriveSideways(s_Swerve, false, -6.25).withTimeout(1.5),
        new ResetPoseWithLL(s_Swerve).withTimeout(0.25)

        //,new PIDToElevSafePosition(algeaPivot).withTimeout(2)

        , new L2_Score(elevator, coralHold, coralPivot, algeaPivot)
 
    );

  }
}


