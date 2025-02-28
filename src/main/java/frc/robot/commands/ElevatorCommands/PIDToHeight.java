// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDToHeight extends SequentialCommandGroup {
  /** Creates a new ManualUpDown. */
  public PIDToHeight(Elevator elevator, AlgaePivot algaePivot, double desiredHeight) {

    if (algaePivot.getPivotEncoder() > Constants.AlgaePivot.ENC_REVS_ELEVATOR_SAFE_POSITION){
      addCommands(
      new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ELEVATOR_SAFE_POSITION),
      new WaitCommand(1), //Adjust as needed
      new DangerPIDToHeight(elevator, desiredHeight));
    }
    else{
      addCommands(
      new DangerPIDToHeight(elevator, desiredHeight));
    }

  }
}
