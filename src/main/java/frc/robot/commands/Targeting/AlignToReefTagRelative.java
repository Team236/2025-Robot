// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Swerve s_Swerve;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, Swerve s_Swerve) {
    //SPENCER/MIKE - I think these should be the Targeting Kps, not the AutoConstants Kps.
    //I see the Auto Kps are about 10x the Targeting Kp (not a problem, likel due to units difference),
    // except for Theta (Rotation) Auto is 100x Targeting Kp. 
    //So kP_ROTATION is maybe too small??  Like by a factor of 10??
    xController = new PIDController(Constants.Targeting.KP_TRANSLATION, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Targeting.KP_STRAFE, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.Targeting.KP_ROTATION, 0, 0);  // Rotation
   // xController = new PIDController(Constants.AutoConstants.kPXController, 0.0, 0);  // Vertical movement
   // yController = new PIDController(Constants.AutoConstants.kPYController, 0.0, 0);  // Horitontal movement
   // rotController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

   
//TODO:  DETERMINE THE CONSTANTS BELOW AS FOLLOWS (Currently set to default values)
/* 
 * the setpoints: put your robot on the field where you want it to be at the end of the aligning 
 * (where it is when it scores) then open the limelight web interface and go to the advanced tab 
 * and get the robot position from there
make sure you set the view to “Robot pose in Target Space”
set each setpoint to its relative value in the web interface
x stepoint = TX
y setpoint = TZ*
rot setpoint = RY
*if your camera isn’t centered with your scoring system you would need to have 2 different Y setpt values
 for the right and left reef pipes. if it is centered you can just use the same value but but negative 
 (-Constants.Y_SETPOINT_REEF_ALIGNMENT)
*/
    rotController.setSetpoint(Constants.Targeting.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.Targeting.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.Targeting.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.Targeting.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Targeting.Y_SETPOINT_RIGHT_REEF_ALIGNMENT : -Constants.Targeting.Y_SETPOINT_LEFT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Targeting.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      //TODO:  CAN ADD HERE TO ORIENT WITH LL WHILE TAG IS IN VIEW, BEFORE DRIVING TO TARGET??
      //ORIENT WITH LL AS FOLLOWS:

     // s_Swerve.getLLPose(); //SPENCER/MIKE - CAN WE ADD THIS HERE????
    //  s_Swerve.resetLLPose();//SPENCER/MIKE - CAN WE ADD THIS HERE????
      s_Swerve.drive(new Translation2d(xSpeed, ySpeed), rotValue, false, true);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      s_Swerve.drive(new Translation2d(), 0, false, true);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(), 0, false, true);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
   /*
    DONT_SEE_TAG_WAIT_TIME - how long that we don’t see the tag before we stop the command
    POSE_VALIDATION_TIME - how long we need to be aligned before we finish the command.
    too small of a value might make the robot miss if the pid isn’t perfect and you ahve overshoot, too long just waste time.
    recommend putting this at a high value when tuning to make sure the pid is good and there isnt any overshoot and lower it later.
    */
    return this.dontSeeTagTimer.hasElapsed(Constants.Targeting.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.Targeting.POSE_VALIDATION_TIME);
  }
}