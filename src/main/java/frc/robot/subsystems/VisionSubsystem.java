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
    // Carries all quality metrics so the drivetrain can weight each estimate
    // =========================
    public record VisionEstimate(
        Pose2d pose,
        double timestampSeconds,
        int    tagCount,
        double avgDistanceMeters,
        double avgAmbiguity
    ) {}

    // =========================
    // NOISE FILTER CONFIG
    // =========================
    private static final int    HISTORY_SIZE         = 8;    // poses kept per camera
    private static final double OUTLIER_THRESHOLD    = 0.5;  // meters from median → reject
    private static final double MAX_POSE_JUMP        = 1.0;  // meters between consecutive frames
    private static final double MAX_AMBIGUITY_MULTI  = 0.20; // multi-tag ambiguity cap
    private static final double MAX_AMBIGUITY_SINGLE = 0.15; // tighter cap for single-tag fallback
    // No hard distance limit for single tags — the dynamic std dev formula
    // already widens trust proportionally to distance, so a far single tag
    // gets accepted but weighted loosely rather than dropped entirely.

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
    // Back-left:  X=-0.321, Y=+0.321, Z=0.40m, yaw=135°,  pitch=-15° (tilted up)
    // Back-right: X=-0.321, Y=-0.321, Z=0.40m, yaw=-135°, pitch=-15° (tilted up)
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
    // =========================
    private final PhotonPoseEstimator backLeftEstimator;
    private final PhotonPoseEstimator backRightEstimator;

    // =========================
    // NOISE FILTER STATE
    // Circular history buffer + last accepted pose, per camera
    // =========================
    private final ArrayDeque<Pose2d> leftHistory  = new ArrayDeque<>(HISTORY_SIZE);
    private final ArrayDeque<Pose2d> rightHistory = new ArrayDeque<>(HISTORY_SIZE);
    private Pose2d lastLeftPose  = null;
    private Pose2d lastRightPose = null;

    // =========================
    // CONSTRUCTOR
    // =========================
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
    // Returns ALL valid estimates from both cameras.
    // Caller fuses every entry independently — true multi-camera localization.
    // =========================
    public List<VisionEstimate> getAllEstimates() {

        List<VisionEstimate> out = new ArrayList<>();

        Optional<VisionEstimate> left = processCamera(
            camBackLeft, backLeftEstimator, leftHistory, lastLeftPose, "Left"
        );
        left.ifPresent(e -> {
            out.add(e);
            lastLeftPose = e.pose();
        });

        Optional<VisionEstimate> right = processCamera(
            camBackRight, backRightEstimator, rightHistory, lastRightPose, "Right"
        );
        right.ifPresent(e -> {
            out.add(e);
            lastRightPose = e.pose();
        });

        Logger.recordOutput("Vision/EstimateCount", out.size());

        return out;
    }

    // =========================
    // PROCESS ONE CAMERA
    // Gate → jump filter → median outlier filter → score → emit
    // =========================
    private Optional<VisionEstimate> processCamera(
            PhotonCamera        camera,
            PhotonPoseEstimator estimator,
            ArrayDeque<Pose2d>  history,
            Pose2d              lastPose,
            String              name) {

        var camResult = camera.getLatestResult();
        var est       = estimator.update(camResult);

        if (est.isEmpty()) return Optional.empty();

        List<PhotonTrackedTarget> targets = est.get().targetsUsed;
        if (targets.isEmpty()) return Optional.empty();

        // --- Basic gate: ambiguity + single-tag distance ---
        if (!passesBasicGate(targets)) {
            Logger.recordOutput("Vision/" + name + "/Rejected", "basic gate");
            return Optional.empty();
        }

        Pose2d pose      = est.get().estimatedPose.toPose2d();
        double timestamp = est.get().timestampSeconds;

        // --- Jump filter: reject teleports between consecutive frames ---
        if (lastPose != null) {
            double jump = pose.getTranslation().getDistance(lastPose.getTranslation());
            if (jump > MAX_POSE_JUMP) {
                Logger.recordOutput("Vision/" + name + "/Rejected",
                    "jump " + String.format("%.2f", jump) + "m");
                return Optional.empty();
            }
        }

        // --- Median outlier filter: reject if far from recent pose cloud ---
        // Needs at least 3 samples for a meaningful median
        if (history.size() >= 3) {
            double medX = median(history.stream().mapToDouble(Pose2d::getX).toArray());
            double medY = median(history.stream().mapToDouble(Pose2d::getY).toArray());
            double dev  = Math.hypot(pose.getX() - medX, pose.getY() - medY);
            if (dev > OUTLIER_THRESHOLD) {
                Logger.recordOutput("Vision/" + name + "/Rejected",
                    "outlier " + String.format("%.2f", dev) + "m from median");
                return Optional.empty();
            }
        }

        // --- Update history ---
        if (history.size() >= HISTORY_SIZE) history.pollFirst();
        history.addLast(pose);

        // --- Compute quality metrics for drivetrain weighting ---
        double avgDist = targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(999.0);

        double avgAmb = targets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
            .average().orElse(999.0);

        Logger.recordOutput("Vision/" + name + "/Pose",     pose);
        Logger.recordOutput("Vision/" + name + "/TagCount", targets.size());
        Logger.recordOutput("Vision/" + name + "/AvgDist",  avgDist);
        Logger.recordOutput("Vision/" + name + "/AvgAmb",   avgAmb);

        return Optional.of(
            new VisionEstimate(pose, timestamp, targets.size(), avgDist, avgAmb)
        );
    }

    // =========================
    // BASIC GATE
    // Single-tag fallback: accepted at any distance but uses a tighter ambiguity
    // cap (0.15) since there is no second tag to cross-check the pose against.
    // Multi-tag: slightly relaxed ambiguity cap (0.20) — geometry redundancy
    // compensates for individual tag pose uncertainty.
    // =========================
    private boolean passesBasicGate(List<PhotonTrackedTarget> targets) {

        double ambiguityCap = targets.size() == 1
            ? MAX_AMBIGUITY_SINGLE
            : MAX_AMBIGUITY_MULTI;

        for (var t : targets) {
            if (t.getPoseAmbiguity() > ambiguityCap) return false;
        }

        return true;
    }

    // =========================
    // MEDIAN HELPER
    // =========================
    private double median(double[] vals) {
        Arrays.sort(vals);
        int n = vals.length;
        return (n % 2 == 0)
            ? (vals[n / 2 - 1] + vals[n / 2]) / 2.0
            : vals[n / 2];
    }
}