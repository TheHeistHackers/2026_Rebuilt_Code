package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    
    // 1. Define both cameras (Make sure these names perfectly match the PhotonVision UI)
    private final PhotonCamera frontCamera = new PhotonCamera("RoundedCamera"); 
    private final PhotonCamera backCamera = new PhotonCamera("SquareCamera"); 
    private final PhotonCamera sideCamera = new PhotonCamera("RectangleCamera"); 

    // 2. Define estimators for both
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;
    private final PhotonPoseEstimator sidePoseEstimator;

    private final DriveSubsystem driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // ==========================================
        // CAMERA MEASUREMENTS (Replace with your tape measure values!)
        // ==========================================
        
        //Left is set to minus, I'm not sure if thats how it should be

        // Front Camera: 20cm forward, center of robot, 50cm high, facing straight forward
        Transform3d frontRobotToCam = new Transform3d(
                new Translation3d(0.305, -0.044, 0.438), 
                new Rotation3d(0, 0, 0)
        );

        // Back Camera: 20cm backward (-0.2), center, 50cm high, facing backward (Math.PI)
        Transform3d backRobotToCam = new Transform3d(
                new Translation3d(-0.279, 0.0, 0.425), 
                new Rotation3d(0, 0, Math.PI) // Math.PI is a 180-degree turn
        );

        // Side Camera
        Transform3d sideRobotToCam = new Transform3d(
                new Translation3d(-0.114, 0.228, 0.438), 
                new Rotation3d(0, 0, Math.PI/2) // Math.PI/2 is a 90-degree turn
        );

        // Initialize the estimators
        frontPoseEstimator = new PhotonPoseEstimator(fieldLayout, frontRobotToCam);
        backPoseEstimator = new PhotonPoseEstimator(fieldLayout, backRobotToCam);
        sidePoseEstimator = new PhotonPoseEstimator(fieldLayout, sideRobotToCam);
    }

    @Override
    public void periodic() {
        // Process both cameras every loop
        processCamera(frontCamera, frontPoseEstimator, "Front");
        processCamera(backCamera, backPoseEstimator, "Back");
        processCamera(sideCamera, sidePoseEstimator, "Side");
    }

    /**
     * Helper method to process a single camera's data to prevent duplicate code.
     */
    private void processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, String cameraName) {
        var result = camera.getLatestResult();

        // Feed that result into the multi-tag pose strategy
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

        if (estimatedPose.isEmpty()){
            estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        // If it successfully calculated a position
        if (estimatedPose.isPresent()) {
            Pose2d robotFieldPosition = estimatedPose.get().estimatedPose.toPose2d();
            double imageCaptureTime = estimatedPose.get().timestampSeconds;

            // SEND THE DATA TO THE DRIVETRAIN
            driveSubsystem.addVisionMeasurement(robotFieldPosition, imageCaptureTime);

            // Print it to SmartDashboard to verify which camera is seeing what
            SmartDashboard.putNumber(cameraName + " Camera/Robot X", robotFieldPosition.getX());
            SmartDashboard.putNumber(cameraName + " Camera/Robot Y", robotFieldPosition.getY());
            SmartDashboard.putNumber(cameraName + " Camera/Robot Rot", robotFieldPosition.getRotation().getDegrees());
        }
    }
}