// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CoralPivot;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.CoralHoldCommands.CoralGrab;
import frc.robot.commands.CoralHoldCommands.CoralGrabWithCounter;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2Right extends SequentialCommandGroup {
  /** Creates a new Leg2Right. */
  public Leg2Right(Swerve s_Swerve, CoralHold coralHold, frc.robot.subsystems.CoralPivot coralPivot, Elevator elevator) {
    addCommands(
      Commands.parallel(
        //Bring elevator down while driving sideways
        new CoralResetCount(coralHold),
        new DriveSideways(s_Swerve, false, 73).withTimeout(1),//timeout needed?  causes delay?
        new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT)
        ), 

      Commands.parallel(
        new DriveFwdAndSideAndTurn(s_Swerve, false, 10, 96, -68).withTimeout(3),//62
        new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LOADING).withTimeout(5), //adjust as needed
        new CoralGrabWithCounter(coralHold, Constants.CoralHold.HOLD_SPEED).withTimeout(5),
        new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT)
        )  
    );
    
  }
}

