package frc.robot.subsystems;

import java.util.Optional;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.*;
import org.photonvision.targeting.*;
import org.photonvision.estimation.*;

import edu.wpi.first.apriltag.*;

public class VisionSubsystem extends SubsystemBase {

    // =========================
    // CAMERAS
    // =========================
    private final PhotonCamera camBackLeft  = new PhotonCamera("LeftCam");
    private final PhotonCamera camBackRight = new PhotonCamera("RightCam");

    // =========================
    // FIELD LAYOUT
    // =========================
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

    // =========================
    // ROBOT TO CAMERA TRANSFORMS
    // Back-left corner:  X = -0.321 (back), Y = +0.321 (left),  Z = 0.40m
    // Back-right corner: X = -0.321 (back), Y = -0.321 (right), Z = 0.40m
    // Yaw:   ±135° from forward (facing out the back corners)
    // Pitch: -15°  (tilted upward to see higher tags)
    // =========================
    private final Transform3d robotToBackLeftCam = new Transform3d(
        new Translation3d(-0.321, 0.321, 0.40),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(135))
    );

    private final Transform3d robotToBackRightCam = new Transform3d(
        new Translation3d(-0.321, -0.321, 0.40),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-135))
    );

    // =========================
    // POSE ESTIMATORS
    // FIX: PhotonPoseEstimator no longer takes a PhotonCamera in its constructor
    //      as of PhotonVision 2025+. Pass only the transform.
    // =========================
    private final PhotonPoseEstimator backLeftEstimator;
    private final PhotonPoseEstimator backRightEstimator;

    public VisionSubsystem() {

        backLeftEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackLeftCam
        );

        backRightEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackRightCam
        );

        backLeftEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );

        backRightEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );
    }

    // =========================
    // MAIN PUBLIC API
    // =========================
    public Optional<EstimatedRobotPose> getLatestEstimate() {

        Optional<EstimatedRobotPose> left  = updateEstimator(camBackLeft,  backLeftEstimator);
        Optional<EstimatedRobotPose> right = updateEstimator(camBackRight, backRightEstimator);

        if (left.isPresent() && right.isPresent()) {

            double leftScore  = score(left.get());
            double rightScore = score(right.get());

            return leftScore > rightScore ? left : right;
        }

        return left.isPresent() ? left : right;
    }

    // =========================
    // UPDATE ESTIMATOR
    // FIX: method was missing entirely (called but never defined).
    //      FIX: uses estimator.update(result) — the correct modern API.
    //      Old getEstimatedGlobalPose(previousPose, timestamp) no longer exists.
    // =========================
    private Optional<EstimatedRobotPose> updateEstimator(
            PhotonCamera camera,
            PhotonPoseEstimator estimator) {

        var result = camera.getLatestResult();
        Optional<EstimatedRobotPose> est = estimator.update(result);

        if (est.isPresent() && !isValid(est.get())) {
            return Optional.empty();
        }

        return est;
    }

    // =========================
    // VALIDATION FILTERS
    // =========================
    private boolean isValid(EstimatedRobotPose est) {

        List<PhotonTrackedTarget> targets = est.targetsUsed;

        if (targets.isEmpty()) return false;

        // Reject single-tag far shots
        if (targets.size() == 1) {
            double dist = targets.get(0).getBestCameraToTarget().getTranslation().getNorm();
            if (dist > 3.5) return false;
        }

        // Reject high ambiguity
        for (var t : targets) {
            if (t.getPoseAmbiguity() > 0.25) return false;
        }

        return true;
    }

    // =========================
    // SCORING FUNCTION
    // =========================
    private double score(EstimatedRobotPose est) {

        double score = 0;

        // More tags = better
        score += est.targetsUsed.size() * 2.0;

        // Closer tags = better
        for (var t : est.targetsUsed) {
            double dist = t.getBestCameraToTarget().getTranslation().getNorm();
            score += Math.max(0, 4 - dist);
        }

        return score;
    }
}