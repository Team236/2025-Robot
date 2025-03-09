// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequencedPIDToL4 extends SequentialCommandGroup {
  /** Creates a new ClimbDownSequence. */
  public SequencedPIDToL4(Elevator elevator, AlgaePivot algaePivot) {
    addCommands(
      //new PIDToElevSafePosition(algaePivot),
      new DangerPIDToHeight(elevator, 6).withTimeout(2),
      new DangerPIDToHeight(elevator, 12).withTimeout(2),
      new DangerPIDToHeight(elevator, 18).withTimeout(2),
      new DangerPIDToHeight(elevator, 24).withTimeout(2),
      new DangerPIDToHeight(elevator, 30).withTimeout(2),
      new DangerPIDToHeight(elevator, 36).withTimeout(2),
      new DangerPIDToHeight(elevator, 42).withTimeout(2),
      new DangerPIDToHeight(elevator, 48).withTimeout(2),
      new DangerPIDToHeight(elevator, 55.25).withTimeout(2)
    );
  }
}
