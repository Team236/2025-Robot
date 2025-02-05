// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //TODO  Change to TalonFX motor controllers (not using SparkMax)
  //private SparkMax leftElevatorMotor, rightElevatorMotor;
  private TalonFX leftElevatorMotor, rightElevatorMotor;
  private TalonFXConfigurator leftConfig, rightConfig;
  private CurrentLimitsConfigs leftCurrentConfigs, rightCurrentConfigs;
  private MotorOutputConfigs leftOutputConfigs, rightOutputConfigs;
  // private SparkMaxConfig leftConfig, rightConfig;

  public RelativeEncoder leftElevEncoder, rightElevEncoder;

  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;

  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_LEFT_TALON);
    rightElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_RIGHT_TALON);
    // leftElevatorMotor = new SparkMax(Constants.MotorControllers.ID_ELEVATOR_LEFT, MotorType.kBrushless);
    // rightElevatorMotor = new SparkMax(Constants.MotorControllers.ID_ELEVATOR_RIGHT, MotorType.kBrushless);

   // leftElevatorMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //rightElevatorMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//CREATE ENCODERS!! New encoder???
    // configure motors
    leftConfig = leftElevatorMotor.getConfigurator();
    leftCurrentConfigs = new CurrentLimitsConfigs();
    leftCurrentConfigs.StatorCurrentLimit = (Constants.MotorControllers.SMART_CURRENT_LIMIT);
    leftOutputConfigs = new MotorOutputConfigs();
    leftOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    leftConfig.apply(leftCurrentConfigs);
    leftConfig.apply(leftOutputConfigs);


    rightConfig = leftElevatorMotor.getConfigurator();
    rightCurrentConfigs = new CurrentLimitsConfigs();
    rightCurrentConfigs.StatorCurrentLimit = (Constants.MotorControllers.SMART_CURRENT_LIMIT);
    rightOutputConfigs = new MotorOutputConfigs();
    rightOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

    rightConfig.apply(leftCurrentConfigs);
    rightConfig.apply(leftOutputConfigs);


   //leftElevEncoder = leftElevatorMotor.getEncoder();
   // rightElevEncoder = rightElevatorMotor.getEncoder();

    try {
      elevatorTopLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_TOP);
    } catch (Exception e) {
      isTException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isTException);
    }

    try {
      elevatorBottomLimit = new DigitalInput(Constants.Elevator.DIO_ELEV_BOTTOM);
    } catch (Exception e) {
      isBException = true;
      SmartDashboard.putBoolean("exception thrown for elev bottom limit: ", isBException);
    }

  }

  public void stopElevator() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  public boolean isETopLimit() {
    if (isTException) {
      return true;
    } else {
      return elevatorTopLimit.get();
    }
  }
  public boolean isEBotLimit() {
    if (isBException) {
      return true;
    } else {
      return elevatorBottomLimit.get();
    }
  }

  // reset/zero encoders
  public void resetElevatorEncoders(){
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS (number of rotations)
  public double getElevLeftEncoder() {
    return leftElevatorMotor.getPosition().getValueAsDouble(); //for a SparkMax encoder
  }
    public double getElevRightEncoder() {
    return rightElevatorMotor.getPosition().getValueAsDouble(); //for a SparkMax encoder
  }

  //returns revolutions of encoders -- NOT CONVERTED TO INCHES
  public double getElevatorHeight(){
    return Constants.Elevator.ELEV_REV_TO_IN * ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
  }

 // check if elevator is at "top" according to user definition
  public boolean isTop() {
    boolean eTop;
    return (getElevatorHeight() >= Constants.Elevator.MAX_HEIGHT);
  }

  public void setElevSpeed(double speed) {
    if (speed > 0) {  
      if (isETopLimit() || isTop()) {
          // if elevator limit is tripped or elevator is near the top limit switch going up, stop 
          stopElevator();
       }  else {
          // elevator going up but top limit is not tripped, go at commanded speed
          leftElevatorMotor.set(speed);
          rightElevatorMotor.set(speed);
        }
      } 
    else {
      if (isEBotLimit()) {
        //elevator going down and is at the bottom,stop and zero encoder
        stopElevator();
        resetElevatorEncoders();
      } else {
      // elevator going down but not at the bottom, go at commanded speed
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
      }
    }
  } 

  public double getElevatorLeftSpeed() {
    return leftElevatorMotor.get();
  }  

  public double getElevatorRightSpeed() {
    return rightElevatorMotor.get();
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator height: ", getElevatorHeight());
    SmartDashboard.putBoolean("Elevator at top? ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom? ", isEBotLimit());
    SmartDashboard.putNumber("Elevator left enc revs = ", getElevLeftEncoder());
    SmartDashboard.putNumber("Elevator right enc revs = ", getElevRightEncoder());
  }
}