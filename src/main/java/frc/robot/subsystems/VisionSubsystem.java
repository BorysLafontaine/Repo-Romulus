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
    private final PhotonCamera camFront = new PhotonCamera("frontCam");
    private final PhotonCamera camBack  = new PhotonCamera("backCam");

    // =========================
    // FIELD LAYOUT
    // =========================
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

    // =========================
    // ROBOT TO CAMERA TRANSFORMS
    // ⚠️ MUST BE MEASURED PRECISELY
    // =========================
    private final Transform3d robotToFrontCam = new Transform3d(
        new Translation3d(0.25, 0.0, 0.20),
        new Rotation3d(0, 0, 0)
    );

    private final Transform3d robotToBackCam = new Transform3d(
        new Translation3d(-0.25, 0.0, 0.20),
        new Rotation3d(0, Math.PI, Math.PI)
    );

    // =========================
    // POSE ESTIMATORS
    // FIX: PhotonPoseEstimator no longer takes a PhotonCamera in its constructor
    //      as of PhotonVision 2025+. Pass only the transform.
    // =========================
    private final PhotonPoseEstimator frontEstimator;
    private final PhotonPoseEstimator backEstimator;

    public VisionSubsystem() {

        frontEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToFrontCam
        );

        backEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackCam
        );

        // fallback if multi-tag fails
        frontEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );

        backEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );
    }

    // =========================
    // MAIN PUBLIC API
    // =========================
    public Optional<EstimatedRobotPose> getLatestEstimate() {

        Optional<EstimatedRobotPose> front = updateEstimator(camFront, frontEstimator);
        Optional<EstimatedRobotPose> back  = updateEstimator(camBack,  backEstimator);

        // =========================
        // BEST SELECTION LOGIC
        // =========================
        if (front.isPresent() && back.isPresent()) {

            double frontScore = score(front.get());
            double backScore  = score(back.get());

            return frontScore > backScore ? front : back;
        }

        return front.isPresent() ? front : back;
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