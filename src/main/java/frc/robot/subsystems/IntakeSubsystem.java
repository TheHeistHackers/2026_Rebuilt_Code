// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;
  private SparkMax intakePistonMotorOne;
  private SparkMax intakePistonMotorTwo;

  public IntakeSubsystem() {

    //Configure the intake
    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);
    intakePistonMotorOne = new SparkMax(Constants.IntakeConstants.intakePistonMotorOneID, MotorType.kBrushless);
    intakePistonMotorTwo = new SparkMax(Constants.IntakeConstants.intakePistonMotorTwoID, MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    //  Find the needed parameters
    // I added the most common ones that all FRC teams use, but the values may need to change

    //Set the brake for when not receiving a command
    intakeConfig.idleMode(IdleMode.kCoast);

    //Smart cutoff, the motor will try to keep this amount of Amperes
    intakeConfig.smartCurrentLimit(35);
    //The main cutoff - it will never go above it. Hardware breaker is 40A, so set software breaker just below
    intakeConfig.secondaryCurrentLimit(39);
    //Stabilizes the torgue so that when the battery is fresh it works the same way as if it's not
    intakeConfig.voltageCompensation(12.0);


    //This line makes sure the settings are persistant and that we reset the controller to safe defaults
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void runIntakeForward(double speed){
        System.out.println("intaking");

    intakeMotor.set(-speed);
  }

  
  public void runIntakeReverse(double speed){
    intakeMotor.set(speed);
  }

  public void extendIntake(){
    System.out.println("extending");
    intakePistonMotorOne.set(-0.5);
    intakePistonMotorTwo.set(0.5);
  }

  public void retractIntake(){
    intakePistonMotorOne.set(0.5);
    intakePistonMotorTwo.set(-0.5);
  }

  public void stopExtendIntake(){
    intakePistonMotorOne.set(0);
    intakePistonMotorTwo.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}