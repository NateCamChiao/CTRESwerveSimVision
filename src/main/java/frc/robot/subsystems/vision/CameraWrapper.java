package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
    private Optional<VisionMeasurement> cachedLastMeasurement = Optional.empty();
    public class VisionMeasurement {
        private EstimatedRobotPose measurementInfo; // includes timestamp and pose
        private final Matrix<N3, N1> standardDeviations = VecBuilder.fill(9, 9, 1000);

        public void updateMeasurement(EstimatedRobotPose newEstimate, double translationStdDevs, double rotationStdDevs){
            this.measurementInfo = newEstimate;
            // edit standardDeviation object
            if(translationStdDevs <= 0)
                translationStdDevs = 0.1;
            if(rotationStdDevs <= 0)
                rotationStdDevs = 0.1;
            this.standardDeviations.set(0,0,translationStdDevs); // x stdDevs
            this.standardDeviations.set(1,0,translationStdDevs); // y stdDevs
            this.standardDeviations.set(2,0,rotationStdDevs);
        }

        public EstimatedRobotPose getMeasurementInfo(){
            return this.measurementInfo;
        }
        
        public Matrix<N3, N1> getStandardDeviations(){
            return this.standardDeviations;
        }
    }

    public CameraWrapper(String cameraName, AprilTagFieldLayout tagLayout, Transform3d robotToCamera,
            SimCameraProperties simCameraProperties) {
        this.camera = new PhotonCamera(cameraName);
        this.poseEstimator = new PhotonPoseEstimator(tagLayout, robotToCamera);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);
        this.robotToCamera = robotToCamera;
    }

    // get the latest unread measurement (for odometry) 
    // should be called periodically to update last measurement
    public Optional<VisionMeasurement> getLatestUnreadMeasurement() {
        Optional<VisionMeasurement> latestPose = Optional.empty();
        var results = camera.getAllUnreadResults(); // earliest to latest results
        // if (results.size() >= 2) {
            // DogLog.log("0th index","" +
            // results.get(0).getTimestampSeconds() + ", " + results.get(results.size() -
            // 1).getTimestampSeconds());
        // }

        if (results.size() <= 0) {
            return Optional.empty();
        }
        PhotonPipelineResult latestResult = results.get(results.size() - 1); // grab the latest result (last in list)
        // check if has targets before anything else to optimize loop
        if (!latestResult.hasTargets()) {
            return Optional.empty();
        }

        if (latestResult.getTargets().size() == 1 && latestResult.getBestTarget().getPoseAmbiguity() < 0.2) {
            // estimatedPose should get allocated to stack bc it doesn't escape the scope
            EstimatedRobotPose estimatedPose = poseEstimator.estimatePnpDistanceTrigSolvePose(latestResult).get(); 

            double translationStdDevs = getPositionStandardDeviations(estimatedPose, latestResult);
            this.mutableMeasurement.updateMeasurement(estimatedPose, translationStdDevs, 1000);
            return Optional.of(this.mutableMeasurement);
            // set rotation standard deviation very low to avoid feedback loop (since we use
            // gyro to help solve)
        } else if (latestResult.getTargets().size() >= 2 && latestResult.getMultiTagResult().isPresent()) {
            EstimatedRobotPose estimatedPose = poseEstimator.estimateCoprocMultiTagPose(latestResult).get();

            double translationStdDevs = getPositionStandardDeviations(estimatedPose, latestResult);
            this.mutableMeasurement.updateMeasurement(estimatedPose, translationStdDevs, 1);
            return Optional.of(mutableMeasurement);
        }
        
        // update cache when new measurement is taken
        if(latestPose.isPresent())
            cachedLastMeasurement = latestPose;
        
        return latestPose;
    }

    public double getPositionStandardDeviations(EstimatedRobotPose robotPose, PhotonPipelineResult result){
        // Pose present. Start running Heuristic
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : result.targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(robotPose.estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            return 9999; 
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            // Increase std devs based on (average) distance
            if (numTags == 1 && avgDist > 4)
                return 9999;
            else {
                return 1 + (avgDist * avgDist / 30);
            }
        }
    }

    // gets last measurement
    public Optional<VisionMeasurement> getLastMeasurement(){
        return this.cachedLastMeasurement;
    }

    public void setHeading(Rotation2d rotation, double timestamp) {
        this.poseEstimator.addHeadingData(timestamp, rotation);
    }

    public void setCameraDisable(boolean shouldDisable) {
        if(shouldDisable){
            this.camera.setFPSLimit(1);
            this.camera.setPipelineIndex(1);
        }
        else{
            this.camera.setFPSLimit(-1); // set to unlimited frames
            this.camera.setPipelineIndex(0);
        }
    }
}