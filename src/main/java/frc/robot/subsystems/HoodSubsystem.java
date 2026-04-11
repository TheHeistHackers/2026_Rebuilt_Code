package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.Map;
import java.util.TreeMap;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import frc.robot.Constants;


public class HoodSubsystem extends SubsystemBase{
    // Hood Motor & Sensors



  // Flag for periodic loop
  private boolean isMovingHood = false;
  private SparkMax hoodMotor; 
  private final DutyCycleEncoder hoodEncoder;
  private final PIDController hoodController;
  private final TreeMap<Double, Double> hoodTable = new TreeMap<>();

  
  public HoodSubsystem(){

    hoodTable.put(0.0, 0.51);
    hoodTable.put(1.0, 0.50);
    hoodTable.put(2.0, 0.48);
    hoodTable.put(3.0, 0.46);
    hoodTable.put(4.0, 0.44);
    hoodTable.put(5.0, 0.42);
    hoodTable.put(6.0, 0.45);

    // ==========================================
    // 2. HOOD ENCODER & PID INITIALIZATION
    // ==========================================
    hoodMotor = new SparkMax(Constants.TurretConstants.hoodMotorID, MotorType.kBrushless);

    hoodEncoder = new DutyCycleEncoder(
      Constants.TurretConstants.hoodEncoderDIOPort, 
      1.0, 
      Constants.TurretConstants.hoodEncoderOffset
    );
    
    hoodController = new PIDController(1.5, 0.0, 0.0);
    hoodController.setTolerance(0.01); 

    // ==========================================
    // 3. HOOD MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    
    hoodConfig.idleMode(IdleMode.kBrake); 
    hoodConfig.smartCurrentLimit(20);     
    hoodConfig.inverted(false); 

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  public double calculateInterpolatedHoodPosition(double currentDistance) {
    // Find the closest table entry below our distance, and the closest above
    Map.Entry<Double, Double> floor = hoodTable.floorEntry(currentDistance);
    Map.Entry<Double, Double> ceiling = hoodTable.ceilingEntry(currentDistance);

    // Edge cases: If we are entirely outside the table limits, snap to the nearest boundary
    if (floor == null && ceiling == null) return 0.0; 
    if (floor == null) return ceiling.getValue();     
    if (ceiling == null) return floor.getValue();     
    if (floor.getKey().equals(ceiling.getKey())) return floor.getValue(); 

    // Weighted Average Math (Linear Interpolation)
    // t is the percentage of how close we are to the ceiling value vs the floor value
    double t = (currentDistance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
    double interpolatedValue = floor.getValue() + t * (ceiling.getValue() - floor.getValue());
    
    return interpolatedValue;
}

public void adjustClosestTableValues(double currentDistance, double adjustmentAmount) {
    Map.Entry<Double, Double> floor = hoodTable.floorEntry(currentDistance);
    Map.Entry<Double, Double> ceiling = hoodTable.ceilingEntry(currentDistance);

    if (floor != null) {
        hoodTable.put(floor.getKey(), floor.getValue() + adjustmentAmount);
    }
    // Only adjust ceiling if it's not the exact same point as the floor!
    if (ceiling != null && (floor == null || !floor.getKey().equals(ceiling.getKey()))) {
        hoodTable.put(ceiling.getKey(), ceiling.getValue() + adjustmentAmount);
    }
}

  // ==========================================
  // HOOD COMMANDS
  // ==========================================
  public void raiseHood() {
    System.out.println("Raising Hood - max");
    hoodController.setSetpoint(0.38);
    isMovingHood = true;
  }

  public void setHoodPosition(double position) {
    System.out.println("Setting Hood Position to: " + position);
    if(position < 0.38 || position > 0.51) {
      System.out.println("Error: Position must be between 0.38 and 0.51");
      return;
    }
    hoodController.setSetpoint(position);
    isMovingHood = true;
  }

  public void lowerHood() {
    System.out.println("Lowering Hood");
    hoodController.setSetpoint(0.51);
    isMovingHood = true;
  }

  public void stopHood() {
    isMovingHood = false;
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {

    for (Map.Entry<Double, Double> entry : hoodTable.entrySet()) {
        SmartDashboard.putNumber("Hood Tuning Table/" + entry.getKey() + "m", entry.getValue());
    }

    // --- HOOD PID LOGIC ---
    double currentHoodPos = hoodEncoder.get(); 

    if (isMovingHood) {
      double motorPower = hoodController.calculate(currentHoodPos);

      if (hoodController.atSetpoint()) {
          stopHood();
      } else {
          hoodMotor.set(MathUtil.clamp(motorPower, -0.4, 0.4));
      }
    }

    // =========================================================
    // SMARTDASHBOARD DEBUGGING
    // =========================================================
    
    // Hood Data
    SmartDashboard.putNumber("Hood/Current_Position", currentHoodPos);
    SmartDashboard.putBoolean("Hood/Is_Moving", isMovingHood);
    SmartDashboard.putNumber("Hood/Target_Setpoint", hoodController.getSetpoint());
    SmartDashboard.putNumber("Hood/Position_Error", hoodController.getPositionError());
    SmartDashboard.putNumber("Hood/Applied_Motor_Power", hoodMotor.get());
  }
}
