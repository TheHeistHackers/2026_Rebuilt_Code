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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final double EXTENDED_POS = 0.38; // Example target 
  private final double RETRACTED_POS = 0.02; // Example target


  private final Timer movementTimer = new Timer(); // 1. Added Timer object
  private final double MOVEMENT_TIMEOUT = 7.0;    // 2. Define the 7s limit

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
    extendController = new PIDController(1, 0.0, 0.0);
    // Set how close it needs to get to the target before stopping (e.g., 0.01 rotations)
    extendController.setTolerance(0.03); 

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

    extendConfig.inverted(true);

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
    
    // 3. Reset and start timer whenever a move starts
    movementTimer.reset();
    movementTimer.start();
    
    isMovingToPosition = true;
  }

  public void retractIntake(){
    System.out.println("Retracting to zero");
    extendController.setSetpoint(RETRACTED_POS);
    
    // 3. Reset and start timer whenever a move starts
    movementTimer.reset();
    movementTimer.start();
    
    isMovingToPosition = true;
  }

  /**
   * Stops the extension motor and disables the PID loop
   */
  public void stopExtension() {
    isMovingToPosition = false;
    movementTimer.stop(); // 4. Stop the timer
    intakeExtendMotor.set(0);
  }

  @Override
  public void periodic() {
    if (isMovingToPosition) {
      
      // 5. Check for timeout
      if (movementTimer.hasElapsed(MOVEMENT_TIMEOUT)) {
          System.out.println("Intake Move TIMEOUT reached! Stopping.");
          stopExtension();
          return; // Exit early so we don't run the PID logic below
      }

      double currentPos = extendEncoder.get(); 
      double motorPower = extendController.calculate(currentPos);

      if (extendController.atSetpoint()) {
          stopExtension();
      } else {
          intakeExtendMotor.set(MathUtil.clamp(motorPower, -0.5, 0.5));
      }
    }

    // =========================================================
    // SMARTDASHBOARD DEBUGGING
    // =========================================================
    // Note: Adding "Intake/" before the names automatically groups them 
    // into a neat little folder in Shuffleboard and Glass!

    // Sensor Data
    SmartDashboard.putNumber("Intake/Current_Position", extendEncoder.get());
    
    // PID State
    SmartDashboard.putBoolean("Intake/Is_Moving", isMovingToPosition);
    SmartDashboard.putNumber("Intake/Target_Setpoint", extendController.getSetpoint());
    
    // Position Error: This is (Setpoint - Current_Position). 
    // Graphing this in Shuffleboard is the easiest way to tune P, I, and D! 
    // You want to see this line hit 0 as smoothly and quickly as possible.
    SmartDashboard.putNumber("Intake/Position_Error", extendController.getPositionError());
    SmartDashboard.putBoolean("Intake/At_Setpoint", extendController.atSetpoint());

    // Hardware Outputs
    // .get() returns the actual clamped power currently being sent to the motor (-1.0 to 1.0)
    SmartDashboard.putNumber("Intake/Applied_Motor_Power", intakeExtendMotor.get());

    // You can remove the System.out.println to prevent console spam, 
    // as SmartDashboard is much easier to read!


    SmartDashboard.putNumber("Intake/Move_Timer", movementTimer.get());
  }
}