// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.PIDToHeight;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1_Score extends SequentialCommandGroup {
  /** Creates a new L1_Score. */
  public L1_Score(Elevator elevator, CoralHold coralHold, CoralPivot coralPivot, AlgaePivot algaePivot) {

    addCommands(

       new PIDToHeight(elevator, algaePivot, Constants.Elevator.L1_HEIGHT),
       new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL1),
       new CoralRelease(coralHold, Constants.CoralHold.L1_RELEASE_SPEED)
    );

//OR, IF CORAL WON'T HIT ELEVATOR:
   //Commands.parallel(
   //   new PIDToHeight(elevator, algaePivot, Constants.Elevator.L1_HEIGHT),
   //   new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL1)
   // ),
   //new CoralRelease(coralHold, Constants.CoralHold.L1_RELEASE_SPEED)
   //);

  }
}