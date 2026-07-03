package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class CameraWrapper {
    public final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    public final PhotonCameraSim cameraSim;
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    public final Transform3d robotToCamera;
    private VisionMeasurement mutableMeasurement = new VisionMeasurement();

    public class VisionMeasurement {
        public EstimatedRobotPose measurementInfo;
        public Matrix<N3, N1> standardDeviations;
    }

    public CameraWrapper(String cameraName, AprilTagFieldLayout tagLayout, Transform3d robotToCamera,
            SimCameraProperties simCameraProperties) {
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);
        this.robotToCamera = robotToCamera;
    }

    public Optional<VisionMeasurement> getLatestPose() {
        Optional<VisionMeasurement> latestPose = Optional.empty();
        var results = camera.getAllUnreadResults();
        if (results.size() >= 2) {
            DogLog.log("0th index",
                    results.get(0).getTimestampSeconds() > results.get(results.size() - 1).getTimestampSeconds());

        }
        double latestTimestamp = 0;
        for (PhotonPipelineResult result : results) {
            // check targets before anything else to optimize loop
            if (result.hasTargets() && result.getTimestampSeconds() > latestTimestamp) {
                latestTimestamp = result.getTimestampSeconds();

                if (result.getTargets().size() == 1) {
                    latestPose = Optional.of(mutableMeasurement);
                    mutableMeasurement.measurementInfo = poseEstimator.estimatePnpDistanceTrigSolvePose(result).get();
                    // set rotation standard deviation very low to avoid feedback loop (since we use
                    // gyro to help solve)
                } else if (latestPose.isEmpty() || result.getTargets().size() >= 2) {
                    latestPose = Optional.of(mutableMeasurement);
                    mutableMeasurement.measurementInfo = poseEstimator.estimateCoprocMultiTagPose(result).get();
                }
            }
        }
        return latestPose;

    }

    public void setHeading(Rotation2d rotation, double timestamp) {
        this.poseEstimator.addHeadingData(timestamp, rotation);
    }

    public void setCameraDisable(boolean shouldDisable) {
        this.camera.setDriverMode(shouldDisable);
    }
}
