package frc.robot.subsystems;

import java.util.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.*;
import org.photonvision.targeting.*;

import edu.wpi.first.apriltag.*;

public class VisionSubsystem extends SubsystemBase {

    // =========================
    // VISION ESTIMATE RECORD
    // =========================
    public record VisionEstimate(
        Pose2d pose,
        double timestampSeconds,
        int    tagCount,
        double avgDistanceMeters,
        double avgAmbiguity
    ) {}

    // =========================
    // CAMERAS
    // =========================
    private final PhotonCamera camBackLeft  = new PhotonCamera("LeftCam");
    private final PhotonCamera camBackRight = new PhotonCamera("RightCam");

    // =========================
    // FIELD LAYOUT
    // Field: FE-2026 Rebuilt Welded  651.22" × 317.69" = 16.541m × 8.069m
    // =========================
    private final AprilTagFieldLayout fieldLayout =
        AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

    // =========================
    // ROBOT-TO-CAMERA TRANSFORMS
    // Cameras are mounted at the back of the robot, angled 45° outward.
    // WPILib yaw convention: 0°=forward, 90°=left, 180°=back, 270°=right (CCW positive).
    // 45° left of backward  = 135° → LeftCam
    // 45° right of backward = 225° → RightCam
    // =========================
    private final Transform3d robotToBackLeftCam = new Transform3d(
        new Translation3d(-0.321, 0.321, 0.42),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(135))
    );

    private final Transform3d robotToBackRightCam = new Transform3d(
        new Translation3d(-0.321, -0.321, 0.42),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(225))
    );

    // =========================
    // POSE ESTIMATORS
    // =========================
    private final PhotonPoseEstimator backLeftEstimator;
    private final PhotonPoseEstimator backRightEstimator;

    // =========================
    // CONSTRUCTOR
    // =========================
    public VisionSubsystem() {

        backLeftEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackLeftCam
        );
        backLeftEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );

        backRightEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBackRightCam
        );
        backRightEstimator.setMultiTagFallbackStrategy(
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
        );
    }

    // =========================
    // REFERENCE POSE (updated each cycle from drivetrain)
    // Lets the single-tag estimator pick the correct pose solution.
    // Without this, LOWEST_AMBIGUITY returns one of two mirror solutions randomly.
    // =========================
    public void setReferencePose(Pose2d pose) {
        backLeftEstimator.setReferencePose(pose);
        backRightEstimator.setReferencePose(pose);
    }

    // =========================
    // SINGLE-TAG AMBIGUITY THRESHOLD
    // PhotonVision ambiguity < 0.2 → pose is trustworthy enough to use.
    // 0.2–1.0 → two solutions are too similar to distinguish reliably → reject.
    // =========================
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

    // =========================
    // MAIN PUBLIC API
    // Returns every valid estimate — filters single-tag high-ambiguity poses.
    // =========================
    public List<VisionEstimate> getAllEstimates() {

        List<VisionEstimate> out = new ArrayList<>();

        processCamera(camBackLeft,  backLeftEstimator,  "Left" ).ifPresent(out::add);
        processCamera(camBackRight, backRightEstimator, "Right").ifPresent(out::add);

        Logger.recordOutput("Vision/EstimateCount", out.size());

        return out;
    }

    // =========================
    // BEST ESTIMATE (Start-button hard reset)
    // Multi-tag PNP is unambiguous → strongly prefer it for hard resets.
    // Fall back to lowest-ambiguity single-tag only if no multi-tag available.
    // =========================
    public Optional<VisionEstimate> getHighConfidenceEstimate() {
        List<VisionEstimate> estimates = getAllEstimates();

        // First preference: multi-tag estimate with best (lowest) ambiguity
        Optional<VisionEstimate> multiTag = estimates.stream()
            .filter(e -> e.tagCount() >= 2)
            .min(Comparator.comparingDouble(VisionEstimate::avgAmbiguity));
        if (multiTag.isPresent()) return multiTag;

        // Fallback: single-tag only if ambiguity is clearly trustworthy
        return estimates.stream()
            .filter(e -> e.avgAmbiguity() < 0.15)
            .min(Comparator.comparingDouble(VisionEstimate::avgAmbiguity));
    }

    // =========================
    // PROCESS ONE CAMERA
    // getAllUnreadResults() returns only frames not yet consumed — avoids the
    // estimator silently skipping repeated getLatestResult() calls with the
    // same timestamp. We take the most recent unread frame.
    // =========================
    private Optional<VisionEstimate> processCamera(
            PhotonCamera        camera,
            PhotonPoseEstimator estimator,
            String              name) {

        var unread = camera.getAllUnreadResults();
        if (unread.isEmpty()) return Optional.empty();

        var est = estimator.update(unread.get(unread.size() - 1));
        if (est.isEmpty()) return Optional.empty();

        List<PhotonTrackedTarget> targets = est.get().targetsUsed;
        if (targets.isEmpty()) return Optional.empty();

        Pose2d pose      = est.get().estimatedPose.toPose2d();
        double timestamp = est.get().timestampSeconds;

        // Sanity: discard poses outside the field boundary
        if (pose.getX() < 0 || pose.getX() > 16.54 ||
            pose.getY() < 0 || pose.getY() > 8.07) {
            Logger.recordOutput("Vision/" + name + "/Rejected", "out of bounds");
            return Optional.empty();
        }

        double avgDist = targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(999.0);

        double avgAmb = targets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .average().orElse(999.0);

        // Single-tag has two mirror pose solutions.
        // Reject if: (a) ambiguity too high to tell solutions apart,
        //            (b) tag is far — perspective error grows with distance.
        if (targets.size() == 1) {
            if (avgAmb > MAX_SINGLE_TAG_AMBIGUITY) {
                Logger.recordOutput("Vision/" + name + "/Rejected", "high ambiguity " + avgAmb);
                return Optional.empty();
            }
            if (avgDist > 4.0) {
                Logger.recordOutput("Vision/" + name + "/Rejected", "single-tag too far " + avgDist);
                return Optional.empty();
            }
        }

        Logger.recordOutput("Vision/" + name + "/Pose",     pose);
        Logger.recordOutput("Vision/" + name + "/TagCount", targets.size());
        Logger.recordOutput("Vision/" + name + "/AvgDist",  avgDist);
        Logger.recordOutput("Vision/" + name + "/AvgAmb",   avgAmb);

        return Optional.of(
            new VisionEstimate(pose, timestamp, targets.size(), avgDist, avgAmb)
        );
    }
}
