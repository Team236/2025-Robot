// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.DangerPIDToHeight;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4_Score extends SequentialCommandGroup {
  /** Creates a new L4_Score. */
  public L4_Score(Elevator elevator, CoralHold coralHold, CoralPivot coralPivot, AlgaePivot algaePivot) {
    addCommands(
    //new PIDToElevSafePosition(algaePivot),
  
    new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5),
    new DangerPIDToHeight(elevator, Constants.Elevator.L4_HEIGHT).withTimeout(1),
    new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4).withTimeout(0.5),
    new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED).withTimeout(0.5),
    new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.5),
   
    //new PIDToElevSafePosition(algaePivot).withTimeout(2),

    new DangerPIDToHeight(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(2)

    );
   
  }
}