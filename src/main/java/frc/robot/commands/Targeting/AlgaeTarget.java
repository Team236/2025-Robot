// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeTarget extends SequentialCommandGroup {

  private double pipeline = 0; 
  private double initialPoseFwd, initialPoseSide, initialPoseAngle;
  private double moveFwd, moveSide, turn;
  private double tv;

 //This command moves robot center to be centered on the AprilTag (assumes camera is to the left, but may be to right on 2025 robot).

 //ALGAETARGET AND CORALLEFT WILL BE THE SAME -USE WHICHEVER ONE WORKS 
 //CAN REMOVE standoffSideways from arguments if this one works

  public AlgaeTarget(Swerve s_Swerve,  double standoffSideways) {

    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

    initialPoseFwd = LimelightHelpers.getTargetPose_CameraSpace("limelight")[2];
    initialPoseSide = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
    initialPoseAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

     //tv =1 means Limelight sees a target
    if (tv == 1) {
      //move forward a distance equal to the FWD distance between the tag and LL camera
      moveFwd = initialPoseFwd;
      //move sideways a distance that will bring center of robot to the center of tag (to score Algae)
      moveSide = initialPoseSide + Constants.Targeting.DIST_CAM_TO_ROBOT_CENTER; //TODO - MAY NEED TO INVERT!!!!!!!
      //turninvert  angle - invert since angle is positive when the target is to the right of the crosshair
      turn = -initialPoseAngle;
    } 
    else {
      //Limelight can't see target, so don't move 
      moveFwd = 0;
      moveSide =0;
      turn = 0;
    };

    addCommands(
   // new TargetAngle(s_Swerve).withTimeout(2),
    new DriveFwdAndSideAndTurn(s_Swerve, false, moveFwd, moveSide, turn).withTimeout(2)
    ); 
  }
}

