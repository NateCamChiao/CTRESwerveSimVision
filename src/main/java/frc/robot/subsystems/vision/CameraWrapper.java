package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraWrapper {
    public final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    public final PhotonCameraSim cameraSim;
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    public final Transform3d robotToCamera;

    public CameraWrapper(String cameraName, AprilTagFieldLayout tagLayout, Transform3d robotToCamera, SimCameraProperties simCameraProperties){
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);
        this.robotToCamera = robotToCamera;
    }

    public Optional<EstimatedRobotPose> getLatestPose(){
        Optional<EstimatedRobotPose> latestPose = Optional.empty();
        double latestTimestamp = 0;
        for(PhotonPipelineResult result: camera.getAllUnreadResults()){
            if(result.getTimestampSeconds() > latestTimestamp){
                latestTimestamp = result.getTimestampSeconds();

                if(result.getTargets().size() == 1){
                    latestPose = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
                }
                else if(latestPose.isEmpty() || result.getTargets().size() >= 2){
                    latestPose = poseEstimator.estimateCoprocMultiTagPose(result);
                }
            }
        }
        return latestPose;
    }

    public void setHeading(Rotation2d rotation, double timestamp){
        this.poseEstimator.addHeadingData(timestamp, rotation);
    }

    public void setCameraDisable(boolean shouldDisable){
        this.camera.setDriverMode(shouldDisable);
    }
}
