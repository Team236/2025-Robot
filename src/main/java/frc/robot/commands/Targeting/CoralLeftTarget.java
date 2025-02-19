// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralLeftTarget extends SequentialCommandGroup {
  /** Creates a new TargetAllSeries. */  
  //Targeting with Limelight
  public CoralLeftTarget(Swerve s_Swerve) {
    addCommands(
       //TODO: TRY TargetAllParallel with 0 standoffSideways, and then DriveFwdAndSide 
      //new TargetAllParallel(s_Swerve, 9, 6.5).withTimeout(1.5),
      //new DriveFwd(s_Swerve, false, 9).withTimeout(1).withTimeout(3));
          
      new TargetAllParallel(s_Swerve, 12, 0).withTimeout(2.0),
      new DriveFwdAndSideAndTurn(s_Swerve, false, 10, 1.6, 0).withTimeout(3));
     
  }
}




/* OLD CODE:

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralLeftTarget extends Command {

  private double pipeline = 0; 
  private double initialPoseFwd, initialPoseSide, initialPoseAngle;
  private double moveFwd, moveSide, turn;
  private double tv;
  private Swerve s_Swerve;   

 //This command moves robot center to be centered on the AprilTag (assumes camera is to the left, but may be to right on 2025 robot).

 //ALGAETARGET AND CORALLEFT WILL BE THE SAME -USE WHICHEVER ONE WORKS 
  public CoralLeftTarget(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }
   
  // Called when the command is initially scheduled.
   @Override
   public void initialize() {
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
      //Limelight can't see target, so don't move (must put a small value in here)
       moveFwd = 1;
       moveSide = 0;
       turn = 0;
      };
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new DriveFwdAndSideAndTurn(s_Swerve, false, moveFwd, moveSide, turn).withTimeout(2); 
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //INSERT CODE TO STOP HERE?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return ( poseAngle < 0.5);
     return false;
  }
}

*/