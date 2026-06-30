package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public final String camera1Name = "";
    public final String camera2Name = "";
    private final SimCameraProperties cameraProp = new SimCameraProperties();
    public final CameraWrapper[] cameras = new CameraWrapper[2];
    private final ArrayList<EstimatedRobotPose> latestCameraPoses = new ArrayList<>(this.cameras.length);

    VisionSystemSim visionSim = new VisionSystemSim("main");

    private Supplier<Rotation2d> headingSupplier = () -> Rotation2d.kZero; // default value will be zero to prevent null
                                                                           // pointer exception
    private Supplier<Pose2d> poseSupplier = () -> Pose2d.kZero; // default zero pose
    private boolean hasSetSuppliers = false;
    private AddVisionMeasurementConsumer addVisionMeasurement;
    @FunctionalInterface
    public static interface AddVisionMeasurementConsumer {
        public void accept(Pose2d estimatedPose, double timestamp, Matrix<N3, N1> estimatedStdDevs);
    }

    public Vision() {
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        visionSim.addAprilTags(fieldLayout);

        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
        // pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z =
        // 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
        cameras[0] = new CameraWrapper(camera1Name, fieldLayout, robotToCamera, cameraProp);
        cameras[1] = new CameraWrapper(camera2Name, fieldLayout,
                robotToCamera.plus(new Transform3d(1.0, 0.5, 0.5, new Rotation3d(0, 0, Math.PI))), cameraProp);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        for (CameraWrapper camera : this.cameras) {
            visionSim.addCamera(camera.cameraSim, camera.robotToCamera);
        }
    }

    // public Optional<EstimatedRobotPose> getLatestCameraPose(PhotonCamera camera,
    // PhotonPoseEstimator poseEstimator){
    // Optional<EstimatedRobotPose> latestPose = Optional.empty();
    // double latestTimestamp = 0;
    // for(PhotonPipelineResult result: camera.getAllUnreadResults()){
    // if(result.getTimestampSeconds() > latestTimestamp){
    // latestTimestamp = result.getTimestampSeconds();

    // if(result.getTargets().size() == 1){
    // latestPose = poseEstimator.estimatePnpDistanceTrigSolvePose(result);
    // }
    // else{
    // latestPose = poseEstimator.estimateCoprocMultiTagPose(result);
    // }
    // }
    // }
    // return latestPose;
    // }

    public ArrayList<EstimatedRobotPose> getLatestPoses() {
        this.latestCameraPoses.clear(); // very important to prevent old data from polluting arraylist
        for (CameraWrapper camera : this.cameras) {
            Optional<EstimatedRobotPose> pose = camera.getLatestPose();
            if (pose.isPresent()) {
                this.latestCameraPoses.add(pose.get());
            }
        }

        return this.latestCameraPoses;
    }

    public void updateHeadingData() {
        double timestamp = Timer.getFPGATimestamp();
        Rotation2d heading = this.headingSupplier.get();

        for (CameraWrapper camera : this.cameras) {
            camera.setHeading(heading, timestamp);
        }
    }

    public void setSubsystemSuppliers(Supplier<Rotation2d> headingSupplier, Supplier<Pose2d> poseSupplier, AddVisionMeasurementConsumer visionMeasurementConsumer) {
        this.headingSupplier = headingSupplier;
        this.poseSupplier = poseSupplier;
        this.addVisionMeasurement = visionMeasurementConsumer;
        this.hasSetSuppliers = true;
    }

    @Override
    public void periodic() {
        if (!hasSetSuppliers) {
            DogLog.logFault("Vision Suppliers Not Called!");
        }
        updateHeadingData();
        for(EstimatedRobotPose estimate : this.getLatestPoses()){
            this.addVisionMeasurement.accept(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, VecBuilder.fill(0.5, 0.5, 0.5));
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(this.poseSupplier.get());
        visionSim.getDebugField().getObject("setn postitio").setPose(this.poseSupplier.get());
        SmartDashboard.putData("Vision Field", this.visionSim.getDebugField());
    }

    public void disableCameras(boolean shouldDisable) {
        for (CameraWrapper camera : this.cameras) {
            camera.setCameraDisable(shouldDisable);
        }
    }
}
