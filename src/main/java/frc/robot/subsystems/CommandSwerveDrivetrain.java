package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem.VisionEstimate;

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // =========================
    // CONSTANTS
    // =========================

    // Hard rejection: vision pose too far from odometry prediction
    private static final double MAX_VISION_ERROR  = 1.5;  // meters

    // Angular rate above which vision is unreliable (camera motion blur)
    private static final double MAX_ANGULAR_RATE  = 2.0;  // rad/s

    // Std dev base values — scaled dynamically per estimate
    // XY trust = BASE_XY_STD * avgDist / tagCount * (1 + ambiguityScale * avgAmbiguity)
    private static final double BASE_XY_STD       = 0.04;
    private static final double AMBIGUITY_SCALE   = 4.0;
    private static final double DIST_SCALE        = 0.3;  // extra penalty per meter

    // Heading is never trusted from vision alone
    private static final double HEADING_STD       = 9999.0;

    // =========================
    // PREDICTIVE LOCALIZATION CONSTANTS
    // When tags are lost, odometry drifts. When they reappear, we scale up
    // std devs proportionally to how long we were blind, so the Kalman filter
    // is skeptical of measurements that land far from the dead-reckoned prediction.
    // =========================
    private static final double OCCLUSION_DECAY   = 0.4;  // std dev multiplier per second blind
    private static final double MAX_OCCLUSION_MUL = 4.0;  // cap on occlusion penalty

    // =========================
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation  = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds driveRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    // =========================
    // LOGGING
    // =========================
    private final DataLog log = DataLogManager.getLog();

    private final DoubleArrayLogEntry odomLog   = new DoubleArrayLogEntry(log, "/Swerve/OdomPose");
    private final DoubleArrayLogEntry visionLog = new DoubleArrayLogEntry(log, "/Swerve/VisionPose");
    private final DoubleLogEntry      errorLog  = new DoubleLogEntry(log, "/Swerve/VisionError");

    // Last pose that was actually fused (post all checks)
    private volatile Pose2d lastAcceptedVisionPose  = new Pose2d();
    // Last pose attempted (used for Field2d ghost — always live)
    private volatile Pose2d lastAttemptedVisionPose = new Pose2d();

    // =========================
    // PREDICTIVE LOCALIZATION STATE
    // =========================
    private double lastTagTimestampSeconds = 0;
    private boolean hadTagsLastCycle       = false;

    // =========================
    // FIELD2D (Elastic Dashboard)
    // =========================
    private final Field2d field = new Field2d();

    // =========================
    // CONSTRUCTORS
    // =========================
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        DataLogManager.start();
        SmartDashboard.putData("Field", field);
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        DataLogManager.start();
        SmartDashboard.putData("Field", field);
        configureAutoBuilder();
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odomStd,
        Matrix<N3, N1> visionStd,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odomStd, visionStd, modules);
        DataLogManager.start();
        SmartDashboard.putData("Field", field);
        configureAutoBuilder();
    }

    // =========================
    // AUTO BUILDER
    // =========================
    public void configureAutoBuilder() {

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> getState().Pose,
            this::resetPose,
            () -> getState().Speeds,
            (speeds, ff) -> setControl(
                driveRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.02))
            ),
            new PPHolonomicDriveController(
                new PIDConstants(12, 0, 0),
                new PIDConstants(8, 0, 0)
            ),
            config,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
    }

    // =========================
    // COMMAND API
    // =========================
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> setControl(request.get()));
    }

    public Command drive(ChassisSpeeds speeds) {
        return run(() -> setControl(driveRequest.withSpeeds(speeds)));
    }

    public Command getAutoPath(String name) {
        return new PathPlannerAuto(name);
    }

    // =========================
    // PERIODIC
    // =========================
    @Override
    public void periodic() {

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(alliance -> {
                setOperatorPerspectiveForward(
                    alliance == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        Pose2d pose = getState().Pose;

        field.setRobotPose(pose);
        field.getObject("VisionPose").setPose(lastAttemptedVisionPose);
        field.getObject("VisionAccepted").setPose(lastAcceptedVisionPose);

        odomLog.append(new double[]{
            pose.getX(), pose.getY(), pose.getRotation().getRadians()
        });

        double error = pose.getTranslation()
            .getDistance(lastAcceptedVisionPose.getTranslation());

        errorLog.append(error);
    }

    // =========================
    // MULTI-ESTIMATE FUSION
    // Called from RobotContainer with all camera estimates each cycle.
    // Each estimate is fused independently with its own std devs.
    // =========================
    public void addVisionMeasurements(List<VisionEstimate> estimates) {

        boolean hasTagsThisCycle = !estimates.isEmpty();

        // Track occlusion duration for predictive trust scaling
        double now = Utils.fpgaToCurrentTime(Timer.getFPGATimestamp());
        if (hasTagsThisCycle) {
            lastTagTimestampSeconds = now;
        }
        hadTagsLastCycle = hasTagsThisCycle;

        double occlusionSeconds = now - lastTagTimestampSeconds;

        for (VisionEstimate est : estimates) {
            fuseEstimate(est, occlusionSeconds);
        }

        SmartDashboard.putBoolean("Vision/TagsVisible", hasTagsThisCycle);
        SmartDashboard.putNumber("Vision/OcclusionSeconds", occlusionSeconds);
    }

    // =========================
    // FUSE ONE ESTIMATE
    // =========================
    private void fuseEstimate(VisionEstimate est, double occlusionSeconds) {

        lastAttemptedVisionPose = est.pose();

        // --- Angular rate gate ---
        double omega = Math.abs(getState().Speeds.omegaRadiansPerSecond);
        if (omega > MAX_ANGULAR_RATE) return;

        // --- Sample odometry at measurement timestamp for latency compensation ---
        Optional<Pose2d> odomAtTime = samplePoseAt(est.timestampSeconds());
        if (odomAtTime.isEmpty()) return;

        double positionError = est.pose().getTranslation()
            .getDistance(odomAtTime.get().getTranslation());

        // --- Hard rejection ---
        if (positionError > MAX_VISION_ERROR) return;

        // =========================
        // DYNAMIC STD DEVS
        // Tighter trust when: more tags, closer distance, lower ambiguity.
        // Looser trust when: long occlusion (dead-reckoning drift unknown).
        // =========================
        double xyStd = (BASE_XY_STD + est.avgDistanceMeters() * DIST_SCALE)
                     / est.tagCount()
                     * (1.0 + AMBIGUITY_SCALE * est.avgAmbiguity());

        // Occlusion penalty: the longer we were blind, the less we trust
        // the first few measurements back — odometry may have drifted significantly
        double occlusionMultiplier = 1.0 + Math.min(
            occlusionSeconds * OCCLUSION_DECAY,
            MAX_OCCLUSION_MUL
        );
        xyStd *= occlusionMultiplier;

        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, HEADING_STD);

        // --- Log ---
        lastAcceptedVisionPose = est.pose();

        visionLog.append(new double[]{
            est.pose().getX(),
            est.pose().getY(),
            est.pose().getRotation().getRadians()
        });

        SmartDashboard.putNumber("Vision/XYStd",        xyStd);
        SmartDashboard.putNumber("Vision/OcclusionMul", occlusionMultiplier);

        // --- Fuse into Kalman filter ---
        super.addVisionMeasurement(
            est.pose(),
            Utils.fpgaToCurrentTime(est.timestampSeconds()),
            stdDevs
        );
    }

    // =========================
    // OVERRIDES
    // =========================
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}