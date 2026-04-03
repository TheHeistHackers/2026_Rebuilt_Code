// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder; // <-- Updated import
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;
  private SparkMax intakeExtendMotor;

  // <-- Updated object type here
  private RelativeEncoder extendEncoder;
  private SparkClosedLoopController extendController;

  public IntakeSubsystem() {

    // Configure the intake motors
    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    intakeExtendMotor = new SparkMax(Constants.IntakeConstants.intakeExtendMotorID, MotorType.kBrushless);

    // ==========================================
    // 1. ROLLER MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.smartCurrentLimit(35);
    intakeConfig.secondaryCurrentLimit(39);
    intakeConfig.voltageCompensation(12.0);

    // Apply config to the roller motor
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ==========================================
    // 2. EXTENSION MOTOR CONFIGURATION (1/4 Turn Logic)
    // ==========================================
    SparkMaxConfig extendConfig = new SparkMaxConfig();
    
    // Usually, mechanisms that hold a position should be in Brake mode
    extendConfig.idleMode(IdleMode.kBrake); 
    extendConfig.smartCurrentLimit(30);

    // Configure the PID values directly on the config object (Tune these!)
    extendConfig.closedLoop
        .pid(0.1, 0.0, 0.0)
        .outputRange(-1.0, 1.0);

    // Apply config to the extension motor
    intakeExtendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get the encoder and controller objects after configuration
    extendEncoder = intakeExtendMotor.getEncoder();
    extendController = intakeExtendMotor.getClosedLoopController();
  }

  public void runIntakeForward(double speed){
    System.out.println("intaking");
    intakeMotor.set(-speed);
  }
  
  public void runIntakeReverse(double speed){
    intakeMotor.set(speed);
  }

  // Set absolute target points
  private final double EXTENDED_POS = 0.25;
  private final double RETRACTED_POS = 0.0;

  public void extendIntake(){
    System.out.println("Extending to absolute position");
    extendController.setSetpoint(EXTENDED_POS, ControlType.kPosition);
  }

  public void retractIntake(){
    System.out.println("Retracting to zero");
    extendController.setSetpoint(RETRACTED_POS, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}