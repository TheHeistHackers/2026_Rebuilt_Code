// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexSubsystem;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));


// Button A: Run Intake and Index One Forward
// new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
//     .whileTrue(new StartEndCommand(
//         () -> {
//             m_intakeSubsystem.runIntakeForward(0.3);
//             m_indexSubsystem.runIndexOneForward(0.3);
//         },  
//         () -> {
//             m_intakeSubsystem.runIntakeForward(0);
//             m_indexSubsystem.runIndexOneForward(0);
//         },
//         m_intakeSubsystem, m_indexSubsystem // Require both subsystems
//     ));

// // Button B: Run Intake and Index One Reverse
// new JoystickButton(m_driverController, XboxController.Button.kA.value)
//     .whileTrue(new StartEndCommand(
//         () -> {
//             m_intakeSubsystem.runIntakeReverse(0.3);
//             m_indexSubsystem.runIndexOneReverse(0.3);
//         },
//         () -> {
//             m_intakeSubsystem.runIntakeReverse(0);
//             m_indexSubsystem.runIndexOneReverse(0);
//         },
//         m_intakeSubsystem, m_indexSubsystem // Require both subsystems
//     ));

    // Button A: Run Index and Turret Forward
new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new StartEndCommand(
        () -> {
            m_indexSubsystem.runIndexOneForward(0.3);
            m_indexSubsystem.runIndexTwoForward(0.3);
            m_turretSubsystem.shoot(0.5);

        },  
        () -> {
            m_indexSubsystem.runIndexOneForward(0);
            m_indexSubsystem.runIndexTwoForward(0);
            m_turretSubsystem.shoot(0);
        },
        m_indexSubsystem, m_turretSubsystem // Require both subsystems
    ));

new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .whileTrue(new StartEndCommand(
        () -> {
            m_intakeSubsystem.extendIntake();

        },  
        () -> {
           m_intakeSubsystem.stopExtendIntake();
        },
        m_intakeSubsystem // Require both subsystems
    ));

    // new JoystickButton(m_driverController, XboxController.Button.kA.value)
    // // 1. WHAT HAPPENS WHILE THE BUTTON IS HELD
    // .whileTrue(
    //     // Step A: Extend the intake for 2 seconds
    //     new StartEndCommand(
    //         () -> { m_intakeSubsystem.extendIntake(); },  
    //         () -> { m_intakeSubsystem.stopExtendIntake(); }, // Stop extending motors
    //         m_intakeSubsystem
    //     )
    //     .withTimeout(0.5) 
        
    //     // Step B: After 2 seconds, start intaking 
    //     .andThen(
    //         new RunCommand(
    //             () -> { m_intakeSubsystem.runIntakeForward(1);; }, 
    //             m_intakeSubsystem
    //         )
    //     )
    // )
    
    // // 2. WHAT HAPPENS WHEN THE BUTTON IS RELEASED
    // .onFalse(
    //     // Step C: Retract the intake
    //     new StartEndCommand(
    //         () -> { m_intakeSubsystem.retractIntake(); },
    //         () -> { m_intakeSubsystem.stopExtendIntake(); }, // Stop retracting motors
    //         m_intakeSubsystem
    //     )
    //     .withTimeout(0.5) // Give it 2 seconds to retract!
    // );

// // Button B: Run Intake and Index One Reverse
// new JoystickButton(m_driverController, XboxController.Button.kB.value)
//     .whileTrue(new StartEndCommand(
//         () -> {
//             m_indexSubsystem.runIndexOneReverse(0.3);
//             m_indexSubsystem.runIndexTwoReverse(0.3);
//             m_turretSubsystem.shootReverse(0.5);
//         },
//         () -> {
//             m_indexSubsystem.runIndexOneReverse(0);
//             m_indexSubsystem.runIndexTwoReverse(0);
//             m_turretSubsystem.shootReverse(0);
//         },
//         m_indexSubsystem, m_turretSubsystem // Require both subsystems
//     ));
}

public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

}
