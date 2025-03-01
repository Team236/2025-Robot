// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.PIDToHeight;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg3Right extends SequentialCommandGroup {
  /** Creates a new Leg3Right. */
  public Leg3Right(Swerve s_Swerve, Elevator elevator, AlgaePivot algeaPivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
      //TODO  add the commands for scoring and receiving coral

          //First command to drive with odometry and end 9" from bumper to AprilTag, centered on Tag  
          new DriveFwdAndSideAndTurn(s_Swerve, true,120, -16, 6).withTimeout(3.5),
          
          //Use limelight to get exactly 12" from front frame (9" from bumper) to AprilTag
          new TargetAllParallel(s_Swerve, 12, 0).withTimeout(2),

         //**** ADD COMMAND HERE TO RESET POSE WITH LIMELIGHT, BEFORE DRIVING WITH ODOMETRY
         new GetPoseWithLL(s_Swerve),

          //Needs to end  with Limelight camera centered 0.4" to the left of the AprilTag center
          new DriveFwdAndSideAndTurn(s_Swerve, false, 9, 1.6, 0),

          //**** ADD COMMAND HERE TO RESET POSE TO VALUE FROM GetPoseWithLL
          new ResetPoseWithLL(s_Swerve),

          Commands.parallel(
            new PIDToHeight(elevator, algeaPivot, Constants.Elevator.L4_HEIGHT),
            new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4)
           ),

          new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED)

    );
  }

}