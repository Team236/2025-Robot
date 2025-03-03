// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaePivot extends SubsystemBase {
  
  private SparkMax algaePivotMotor;
  private SparkMaxConfig algaePivotConfig;

  private DigitalInput algaeLimit;
  private boolean isPivotException;

  private double desiredSpeed;
  private RelativeEncoder algaePivotEncoder;
  //private boolean hasResetPivotEncoder;

  public AlgaePivot() {
    algaePivotMotor = new SparkMax(Constants.MotorControllers.ID_ALGAE_PIVOT, MotorType.kBrushless);
    algaePivotEncoder = algaePivotMotor.getEncoder();

    algaePivotConfig = new SparkMaxConfig();

    algaePivotConfig.inverted(false);
    algaePivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    //Must do things like invert and set current limits BEFORE callling the motor.configure class below
    algaePivotMotor.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  
    //SmartDashboard.setDefaultBoolean("Algae Digital Input threw an exception", false);

    //hasResetPivotEncoder = false;

    try {
      algaeLimit = new DigitalInput(Constants.AlgaePivot.DIO_LIMIT);
    } catch (Exception e)
    {
      isPivotException = true;
      SmartDashboard.putBoolean("Algae Extend Limit switch threw an exception", true);
    }
 
  }

  //METHODS START HERE
  public boolean isLimit()
  {
    if (isPivotException) {
      return true;
    } else{
      return algaeLimit.get();
    }
  }

  public boolean isFullyExtended(){
    return getPivotEncoder() <= Constants.AlgaePivot.ENC_REVS_MAX;
  }

  public void stopAlgaePivot()
  {
    algaePivotMotor.set(0);
  }

  public void resetPivotEncoder()
  {
    algaePivotEncoder.setPosition(0);
    ///hasResetPivotEncoder = true;
  }

  public double getPivotEncoder()
  {
    return algaePivotEncoder.getPosition();
  }

  public double getPivotSpeed()
  {
    return algaePivotMotor.get();
  }

 // public boolean atRetractLimit(){
 //   if ((getPivotSpeed() >= 0 || desiredSpeed >= 0) && isLimit()){ //positive speed means retracting
 //     return true;
  //     } else {
  //      return false;
  //     }
  //   }

  public void setAlgaePivotSpeed(double speed){  
    desiredSpeed = speed;

    if (speed <= 0){ //negative speed means extending
      //Extending
      if (isFullyExtended()){
        stopAlgaePivot();
      } else {
        algaePivotMotor.set(speed);
      }
    } 
    else 
    {//Retracting
      if (isLimit()){
        //Added line below - assuming we should stop at retract limit - 
        //TODO: remove line blow if needed to hold PID
        stopAlgaePivot();
       // SmartDashboard.putBoolean("AP Limit Hit, so resetting encoder", true);
        resetPivotEncoder();
      } else {
        algaePivotMotor.set(speed);
      }
    }
  }
  
  @Override
  public void periodic() {
   // SmartDashboard.putNumber("Algae Pivot speed is: ", getPivotSpeed());
    SmartDashboard.putBoolean("Algae Pivot is limit hit", isLimit());
    SmartDashboard.putBoolean("Algae Pivot is fully extended", isFullyExtended());
    SmartDashboard.putNumber("Algae Pivot encoder revolutions", getPivotEncoder());
  }
}