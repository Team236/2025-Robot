// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX leftElevatorMotor, rightElevatorMotor;
  private TalonFXConfigurator leftConfig, rightConfig;
  private TalonFXConfiguration leftTalonConfig, rightTalonConfig;

  private CurrentLimitsConfigs leftCurrentConfigs, rightCurrentConfigs;
  private MotorOutputConfigs leftOutputConfigs, rightOutputConfigs;

  private DigitalInput elevatorTopLimit, elevatorBottomLimit;
  private boolean isTException, isBException;

  public Elevator() {
    leftElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_LEFT_TALON, "usb");
    rightElevatorMotor = new TalonFX(Constants.MotorControllers.ID_ELEVATOR_RIGHT_TALON, "usb");

    // configure motors
    leftTalonConfig = new TalonFXConfiguration();
    leftTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    leftTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftElevatorMotor.getConfigurator().apply(leftTalonConfig);

    rightTalonConfig = new TalonFXConfiguration();
    //rightTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    rightTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightElevatorMotor.getConfigurator().apply(rightTalonConfig);

    rightElevatorMotor.setControl(new Follower(Constants.MotorControllers.ID_ELEVATOR_LEFT_TALON, false));

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
   // rightElevatorMotor.set(0);
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
   // rightElevatorMotor.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS (number of rotations)
  public double getElevLeftEncoder() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }
    public double getElevRightEncoder() {
    return rightElevatorMotor.getPosition().getValueAsDouble();
  }

  public double getElevatorHeight(){
    return Constants.Elevator.ELEV_REV_TO_IN * ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
  }

    public double getElevatorHeightMeters(){
      return Constants.Elevator.ELEV_REV_TO_IN * 0.0254 * ((getElevLeftEncoder() + getElevRightEncoder()) / 2);
    }

 // check if elevator is at "top" according to user definition
  public boolean isTop() {
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
         // rightElevatorMotor.set(speed);
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
       // rightElevatorMotor.set(speed);
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
    SmartDashboard.putBoolean("Elevator at top limit: ", isETopLimit());
    SmartDashboard.putBoolean("Elevator at bottom limit: ", isEBotLimit());
    SmartDashboard.putBoolean("Elevator at Max height: ", isTop());
    //SmartDashboard.putNumber("Elevator left enc revs = ", getElevLeftEncoder());
   // SmartDashboard.putNumber("Elevator right enc revs = ", getElevRightEncoder());

  }
}