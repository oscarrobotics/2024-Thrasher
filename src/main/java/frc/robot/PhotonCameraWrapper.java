package frc.robot;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Swerve.SwerveSubsystem;

public class PhotonCameraWrapper {
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    static PhotonCameraWrapper instance; 

    public PhotonCameraWrapper(String cameraName, Transform3d cameraOffset){
        photonCamera = new PhotonCamera(cameraName);

        AprilTagFieldLayout fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        photonPoseEstimator = 
            new PhotonPoseEstimator(
                fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, cameraOffset);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        if(photonPoseEstimator == null){
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public static PhotonCameraWrapper getInstance(){
        if(instance == null){
            instance = new PhotonCameraWrapper(Constants.VisionConstants.cameraName, Constants.VisionConstants.robotToCam);
        }
        return instance;
    }

}
