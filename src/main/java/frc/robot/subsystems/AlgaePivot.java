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

  private DigitalInput algaeExtLimit, algaeRetLimit;
  private boolean isPivotExtException, isPivotRetException;

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
  
    //SmartDashboard.setDefaultBoolean("Algae Extend Digital Input threw an exception", false);
    //SmartDashboard.setDefaultBoolean("Algae Retract Digital Input threw an exception", false);

    //hasResetPivotEncoder = false;

    try {
      algaeExtLimit = new DigitalInput(Constants.AlgaePivot.DIO_EXT_LIMIT);
    } catch (Exception e)
    {
      isPivotExtException = true;
      SmartDashboard.putBoolean("Algae Extend Limit switch threw an exception", true);
    }
    try {
      algaeRetLimit = new DigitalInput(Constants.AlgaePivot.DIO_RET_LIMIT);
    } catch (Exception e)
    {
      isPivotRetException = true;
      SmartDashboard.putBoolean("Algae Retract Limit Switch threw an exception", true);
    }
  }

  //METHODS START HERE

  public boolean isRetLimit()
  {
    if (isPivotRetException)
    {
      return true;
    } else
    {
      return algaeRetLimit.get();
    }
  }

  public boolean isExtLimit(){
    if (isPivotExtException)
    {
      return true;
    } else
    {
      return algaeExtLimit.get();
    }
  }

  public boolean isFullyExtended(){
    return getPivotEncoder() <= Constants.AlgaePivot.ENC_REVS_MAX;
  }

  public void setAlgaePivotSpeed(double speed){  
    if (speed <= 0){  //negative speed means extending
      //Extending
      if (isExtLimit() || isFullyExtended()){
        stopAlgaePivot();
      } else
      {
        algaePivotMotor.set(speed);
      }
    } else 
    {//Retracting
      if (isRetLimit()){
        stopAlgaePivot();
        resetPivotEncoder();
      } else{
        algaePivotMotor.set(speed);
      }
    }
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

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Algae Pivot hit extended limit", isExtLimit());
    SmartDashboard.putBoolean("Algae Pivot hit retracted limit", isRetLimit());
    SmartDashboard.putBoolean("Algae Pivot is fully extended", isFullyExtended());
    SmartDashboard.putNumber("Algae Pivot encoder revolutions", getPivotEncoder());
   // SmartDashboard.putBoolean("Algae Pivot PID controls ready to use: ", hasResetPivotEncoder);
  }
}