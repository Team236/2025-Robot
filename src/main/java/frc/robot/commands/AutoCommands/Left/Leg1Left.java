// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveFwdAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg1Left extends SequentialCommandGroup {
  /** Creates a new Leg1Left. */
  public Leg1Left(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
    //!!!!! TODO MAKE ALL Y DISTANCES AND ALL ANGLES OPPOSITE TO Right !!!!
    addCommands(
      //START ROBOT WITH BACK BUMPER FLUSH WITH BACK OF BLACK STARTING LINE
          //If using odometry plus LL:
          // First command to drive with odometry and end ~7" from bumper to AprilTag, centered on Tag
         new DriveFwdAndTurn(s_Swerve, false, 66.5,  61.4).withTimeout(1.3),
        //  new TargetAngle(s_Swerve).withTimeout(1),
        //  new TargetForwardDistance(s_Swerve, 2).withTimeout(1),
          new TargetSideDistance(s_Swerve, 0).withTimeout(1),
          new GetPoseWithLL(s_Swerve),
          new DriveSideways(s_Swerve, false, 7.25).withTimeout(1.5),
          new DriveFwd(s_Swerve, false, 8.8).withTimeout(2),
          // // new DriveFwdAndSideAndTurn(s_Swerve,false, 3, -6.5, 0).withTimeout(2),
          new ResetPoseWithLL(s_Swerve)
    

      //If using odometry only, starting 96.25" from side (drive to reef with coral scorer aligned to right branch)
        // new DriveFwdAndSideAndTurn(s_Swerve, false, 78.5, 0, -58.2).withTimeout(3)//,
        //new PIDToElevSafePosition(algaePivot).withTimeout(2)   
        //,

    );         


//ADD IN THE SCORING:
       // , new PIDToHeight(elevator, algaePivot, Constants.Elevator.L4_HEIGHT),
        //  new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4),
        //  new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED)
      
      ///OR:
       // , Commands.parallel(
       //    new PIDToHeight(elevator, algaePivot, Constants.Elevator.L4_HEIGHT),
       //    new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4)
       //    ),
        //  new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED)



  }
}
