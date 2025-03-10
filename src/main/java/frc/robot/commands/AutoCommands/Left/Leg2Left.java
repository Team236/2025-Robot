// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.CoralHoldCommands.CoralGrabWithCounter;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2Left extends SequentialCommandGroup {
  /** Creates a new Leg2Left. */
  public Leg2Left(Swerve s_Swerve, CoralHold coralHold, frc.robot.subsystems.CoralPivot coralPivot) {
 //!!!!! TODO MAKE ALL Y DISTANCES AND ALL ANGLES OPPOSITE TO Right !!!!
    addCommands(
        new DriveSideways(s_Swerve, false, -73), //.withTimeout(2),
        new DriveFwdAndSideAndTurn(s_Swerve, false, 5, -115, 68).withTimeout(3),//62

        Commands.parallel(
        new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LOADING).withTimeout(2), //adjust as needed
        new CoralGrabWithCounter(coralHold, Constants.CoralHold.HOLD_SPEED).withTimeout(2) 
        )
    );
  }
}
