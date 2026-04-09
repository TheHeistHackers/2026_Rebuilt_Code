// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Pose Estimator class for tracking robot pose (replaces Odometry)
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d() // Initial starting pose
  );

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

        private final Field2d m_field = new Field2d();


  @Override
  public void periodic() {
    // Update the pose estimator with encoder and gyro data
    m_poseEstimator.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    SmartDashboard.putNumber("Gyro Angle", -m_gyro.getAngle());
    
// Inside your constructor:
SmartDashboard.putData("Field", m_field);

// Inside your periodic() method, add this at the end:
m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());


    // Optional but highly recommended: send your exact position to the dashboard!
    Pose2d currentPose = getPose();
    SmartDashboard.putNumber("Robot X", currentPose.getX());
    SmartDashboard.putNumber("Robot Y", currentPose.getY());
  
  Pose2d towerPose = new Pose2d(8.0, 4.0, new Rotation2d());

    // Update the dashboard with targeting data every 20ms
    outputTargetingData(towerPose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  
  /**
   * Updates the robot's position on the field using vision data.
   * @param visionRobotPoseMeters The calculated X/Y/Rotation from the camera.
   * @param timestampSeconds When the camera actually took the picture.
   */
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
      m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // Sets the wheels into an X formation to prevent movement.
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

public void outputTargetingData(Pose2d targetPose) {
    Pose2d currentPose = getPose();

    // 1. Calculate Distance to Tower (in meters)
    double distanceToTower = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // 2. Calculate the target angle
    Translation2d difference = targetPose.getTranslation().minus(currentPose.getTranslation());
    Rotation2d targetAngle = difference.getAngle();

    // 3. Calculate the rotation error (Shortest path to target)
    // WPILib's .minus() on Rotation2d automatically handles the -180 to 180 boundary
    Rotation2d rotationError = targetAngle.minus(currentPose.getRotation());
    double turnAmountDegrees = rotationError.getDegrees();

    // 4. Determine if we need to turn left
    // In WPILib's coordinate system, counter-clockwise (left) is positive
    boolean turnLeft = turnAmountDegrees > 0;

    // We take the absolute value so "turn amount" is always a positive magnitude
    double absoluteTurnAmount = Math.abs(turnAmountDegrees);


    // 5. Output the three values to SmartDashboard / Elastic
    SmartDashboard.putBoolean("Targeting/Turn Left", turnLeft);
    SmartDashboard.putNumber("Targeting/Turn Amount (Deg)", absoluteTurnAmount);
    SmartDashboard.putNumber("Targeting/Distance to Tower (Meters)", distanceToTower);
  }

  /**
   * Generates a command that drives the robot to a specific Pose2d on the field.
   * * @param targetPose The desired X, Y, and Rotation on the field.
   * @return A Command that handles the PID loop to reach the target.
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    // 1. Create the PID controllers
    // Note: You will need to tune these P values (the "1.0") for your specific robot
    PIDController xController = new PIDController(1.0, 0, 0);
    PIDController yController = new PIDController(1.0, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    // Tell the rotation controller that -180 and 180 degrees are the same place
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances (e.g., within 5cm and 2 degrees)
    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Math.toRadians(2));

    // 2. Return the constructed command
    return this.run(() -> {
      Pose2d currentPose = getPose();

      // Calculate speeds and clamp them between -1.0 and 1.0
      double xSpeed = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.getX()), -1.0, 1.0);
      double ySpeed = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.getY()), -1.0, 1.0);
      double rotSpeed = MathUtil.clamp(
          thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()), 
          -1.0, 1.0);

      // Drive the robot (field-relative is true)
      drive(xSpeed, ySpeed, rotSpeed, true);

    })
    // 3. Stop running when all three PID controllers reach their target
    .until(() -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal())
    // 4. Force the robot to stop moving completely when the command finishes or is interrupted
    .finallyDo(() -> drive(0, 0, 0, true));
  }
// 1. CHANGE THIS LINE: Accept a Supplier<Pose2d> instead of a static Pose2d
  public Command driveAndAimCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Pose2d> targetPoseSupplier) {
    
    // Create a rotation PID controller to snap to the target angle
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.5, 0, 0, // P, I, D (Tune this P value!)
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2) // Max speed and acceleration
    );
    
    // Tell the controller that -180 and 180 degrees are the same
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Return the command that runs every 20ms
    return this.run(() -> {
      // Get the driver's commanded translation speeds
      double xSpeed = xSupplier.getAsDouble();
      double ySpeed = ySupplier.getAsDouble();

      // Get current robot pose
      Pose2d currentPose = getPose();

      // ========================================================
      // 2. ADD THIS LINE: Fetch the correct pose RIGHT NOW.
      // This forces the code to check your alliance color 
      // every single loop while you hold the button!
      // ========================================================
      Pose2d activeTargetPose = targetPoseSupplier.get();

      // Calculate the angle required to point AT the target using the activeTargetPose
      Translation2d difference = activeTargetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d targetAngle = difference.getAngle();

      // Calculate the rotation speed using PID to close the gap
      double rotSpeed = thetaController.calculate(
          currentPose.getRotation().getRadians(), 
          targetAngle.getRadians()
      );

      // Clamp the rotation speed to legal limits [-1.0 to 1.0]
      rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

      // Drive! (Feed joystick translation + automatic rotation)
      drive(xSpeed, ySpeed, rotSpeed, true);
    });
  }
}
