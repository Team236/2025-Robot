// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.AlgaePivotCommands.PIDToSafeAP;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZOLD_ManualUpDown extends SequentialCommandGroup {
  /** Creates a new ManualUpDown. */
  public ZOLD_ManualUpDown(Elevator elevator, AlgaePivot algaePivot, double speed) {
      addCommands(
        new PIDToSafeAP(algaePivot),
        new WaitCommand(5) //Adjust as needed
      //, new DangerManualUpDown(elevator, speed) //ADD BACK IF FIRST COMMAND WORKS!
        );
    }
  }
