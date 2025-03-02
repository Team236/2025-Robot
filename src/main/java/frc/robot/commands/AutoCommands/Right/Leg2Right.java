// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.CoralHoldCommands.CoralGrab;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg2Right extends SequentialCommandGroup {
  /** Creates a new Leg2Right. */
  public Leg2Right(Swerve s_Swerve, CoralHold coralHold) {
    addCommands(
      //TODO  add the commands for scoring and receiving coral

          //Tweak as needed to end against the Coral Loading station, not close to side of field
          //note - camera on 2025 robot will be on opposite side of robot compared with the swerve testbed
          new DriveSideways(s_Swerve, false, 60), //.withTimeout(2),
          new DriveFwdAndSideAndTurn(s_Swerve, false, -3.5, 91.5, -63)

//ADD IN RECEIVING CORAL:
         , new CoralGrab(coralHold, Constants.CoralHold.HOLD_SPEED).withTimeout(2) //adjust as needed
      

    );
  }

}