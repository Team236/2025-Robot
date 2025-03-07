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
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralHoldCommands.CoralReleaseNoCountReset;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.CoralSafePIDToHeight;
import frc.robot.commands.ElevatorCommands.DangerPIDToHeight;
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
      Commands.parallel(
        new DriveFwd(s_Swerve, false, Constants.AutoConstants.CENTER_FWD_DIST).withTimeout(2)
        //,new PIDToElevSafePosition(algeaPivot).withTimeout(2)
        ),

   //SCORE:
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(2),
      //new CoralSafePIDToHeight(elevator, coralHold, Constants.Elevator.L4_HEIGHT).withTimeout(2),
      new DangerPIDToHeight(elevator, Constants.Elevator.L4_HEIGHT).withTimeout(2),
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4).withTimeout(2),
     
      new CoralReleaseNoCountReset(coralHold, Constants.CoralHold.L4_RELEASE_SPEED).withTimeout(0.5),
    
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(2),
       
      //new PIDToElevSafePosition(algeaPivot).withTimeout(2),
    
      //new CoralSafePIDToHeight(elevator, coralHold, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(2),
      new DangerPIDToHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT)//,
      
      //new CoralResetCount(coralHold)
    );

  }
}


