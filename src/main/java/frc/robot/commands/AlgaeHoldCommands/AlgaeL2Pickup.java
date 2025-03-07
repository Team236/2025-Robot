// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeHoldCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.ElevatorCommands.DangerPIDToHeight;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

public class AlgaeL2Pickup extends SequentialCommandGroup {
  /** Creates a new Algae_Score. */
  public AlgaeL2Pickup(Elevator elevator, AlgaeHold algaeHold, AlgaePivot algaePivot) {
  addCommands(
    new PIDMakeAPSafeForElev(algaePivot),
    new DangerPIDToHeight(elevator, Constants.Elevator.PICK_ALGAE_L2_HEIGHT),
    new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_REEF_PICKUP),
    new AlgaeGrab(algaeHold, Constants.AlgaeHold.HOLD_SPEED1, Constants.AlgaeHold.HOLD_SPEED2),
    new PIDMakeAPSafeForElev(algaePivot),
    new DangerPIDToHeight(elevator, Constants.Elevator.TELEOP_HEIGHT)
);

}
}
