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

public class IndexSubsystem extends SubsystemBase {

  private SparkMax indexMotorOne;
  private SparkMax indexMotorTwo;


  public IndexSubsystem() {
    //Configure the index motors
    indexMotorOne = new SparkMax(Constants.IndexConstants.indexMotorOneID, MotorType.kBrushless);
    indexMotorTwo = new SparkMax(Constants.IndexConstants.indexMotorTwoID, MotorType.kBrushless);

    SparkMaxConfig indexConfig = new SparkMaxConfig();

    //  Find the needed parameters
    // I added the most common ones that all FRC teams use, but the values may need to change

    //Set the brake for when not receiving a command
    indexConfig.idleMode(IdleMode.kCoast);

    //Smart cutoff, the motor will try to keep this amount of Amperes
    indexConfig.smartCurrentLimit(35);
    //The main cutoff - it will never go above it. Hardware breaker is 40A, so set software breaker just below
    indexConfig.secondaryCurrentLimit(39);
    //Stabilizes the torgue so that when the battery is fresh it works the same way as if it's not
    indexConfig.voltageCompensation(12.0);


    //This line makes sure the settings are persistant and that we reset the controller to safe defaults
    indexMotorOne.configure(indexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIndexOneForward(double speed){
    indexMotorOne.set(speed);
  }

  public void runIndexOneReverse(double speed){
    indexMotorOne.set(-speed);
  }

  public void runIndexTwoForward(double speed){
    indexMotorTwo.set(speed);
  }

  public void runIndexTwoReverse(double speed){
    indexMotorTwo.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
