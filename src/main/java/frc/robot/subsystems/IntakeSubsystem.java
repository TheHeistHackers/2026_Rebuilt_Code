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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax intakeMotor;
  private SparkMax intakeExtendMotor;

  // roboRIO components for the Absolute Encoder
  private final DutyCycleEncoder extendEncoder;
  private final PIDController extendController;

  // Flag to tell the periodic loop when it should be applying PID power
  private boolean isMovingToPosition = false;

  // IMPORTANT: Since you are now using an absolute encoder, your target points will change.
  // The DutyCycleEncoder returns rotations (e.g., 0.0 to 1.0). 
  // You will need to physically move your mechanism and read the encoder values to find these new numbers!
  private final double EXTENDED_POS = 0.2; // Example target 
  private final double RETRACTED_POS = 0.0; // Example target

  public IntakeSubsystem() {

    // Configure the intake motors
    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    intakeExtendMotor = new SparkMax(Constants.IntakeConstants.intakeExtendMotorID, MotorType.kBrushless);

    // ==========================================
    // 1. ENCODER & PID INITIALIZATION
    // ==========================================
    // Initialize the encoder on the DIO port specified in your Constants (e.g., 0)
    extendEncoder = new DutyCycleEncoder(
      Constants.IntakeConstants.encoderDIOPort, 
  1.0,                                      // Full range (1.0 means 1 full rotation)
      Constants.IntakeConstants.encoderOffset   // The reading where you expect "0.0" to be
);
    
    // Initialize the roboRIO PID Controller (P, I, D) -> You MUST tune these for your mechanism!
    extendController = new PIDController(0.07, 0.0, 0.0);
    // Set how close it needs to get to the target before stopping (e.g., 0.01 rotations)
    extendController.setTolerance(0.01); 

    // ==========================================
    // 2. ROLLER MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig.idleMode(IdleMode.kCoast);
    intakeConfig.smartCurrentLimit(35);
    intakeConfig.secondaryCurrentLimit(39);
    intakeConfig.voltageCompensation(12.0);

    // Apply config to the roller motor
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ==========================================
    // 3. EXTENSION MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig extendConfig = new SparkMaxConfig();
    
    // Brake mode is correct for holding a position
    extendConfig.idleMode(IdleMode.kBrake); 
    extendConfig.smartCurrentLimit(30);
    // Notice: We removed the closedLoop PID configuration from here because 
    // the SparkMax is no longer doing the math, the roboRIO is.

    // Apply config to the extension motor
    intakeExtendMotor.configure(extendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runIntakeForward(double speed){
    System.out.println("intaking");
    intakeMotor.set(-speed);
  }
  
  public void runIntakeReverse(double speed){
    intakeMotor.set(speed);
  }

  public void extendIntake(){
    System.out.println("Extending to absolute position");
    extendController.setSetpoint(EXTENDED_POS);
    isMovingToPosition = true;
  }

  public void retractIntake(){
    System.out.println("Retracting to zero");
    extendController.setSetpoint(RETRACTED_POS);
    isMovingToPosition = true;
  }

  /**
   * Stops the extension motor and disables the PID loop
   */
  public void stopExtension() {
    isMovingToPosition = false;
    intakeExtendMotor.set(0);
  }

  @Override
  public void periodic() {
    // If the subsystem was told to move to a position, run the PID math here
    if (isMovingToPosition) {
      
      // Get the current position from the DIO absolute encoder
      double currentPos = extendEncoder.get(); 
      
      // Calculate how much power the motor needs to reach the setpoint
      double motorPower = extendController.calculate(currentPos);

      // Check if the mechanism has reached the target (within the tolerance set earlier)
      if (extendController.atSetpoint()) {
          stopExtension();
      } else {
          // Clamp the motor output between -0.5 and 0.5 so it doesn't slam into physical stops while you are tuning P, I, and D.
          // You can expand this range to -1.0 and 1.0 once your PID is safely tuned.
          intakeExtendMotor.set(MathUtil.clamp(motorPower, -0.5, 0.5));
      }
    }

      System.out.println("Current Extend Position: " + extendEncoder.get());

  }
}