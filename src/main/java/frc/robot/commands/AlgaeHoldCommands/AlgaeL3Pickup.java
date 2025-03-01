// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeHoldCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.ElevatorCommands.PIDToHeight;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

public class AlgaeL3Pickup extends SequentialCommandGroup {
  /** Creates a new Algae_Score. */
  public AlgaeL3Pickup(Elevator elevator, AlgaeHold algaeHold, AlgaePivot algaePivot) {
  addCommands(
  // Commands.parallel(
  //    new PIDToHeight(elevator, algaePivot, Constants.Elevator.PICK_ALGAE_L3_HEIGHT),
   //   new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ALGAE_PICKUP)
  //  ),
    new PIDToHeight(elevator, algaePivot, Constants.Elevator.PICK_ALGAE_L3_HEIGHT),
    new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ALGAE_PICKUP),
    
    new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED1, Constants.AlgaeHold.HOLD_SPEED2)
    );
  }
}

