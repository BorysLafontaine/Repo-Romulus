package frc.robot.subsystems;

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

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // =========================
    // CONSTANTS (TUNE THESE)
    // =========================
    private static final double MAX_VISION_ERROR = 1.5;  // meters — reject above this
    private static final double SNAP_ERROR       = 0.8;  // meters — hard relocalize above this
    private static final double MAX_ANGULAR_RATE = 2.0;  // rad/s  — reject vision while spinning fast

    // =========================
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation  = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds driveRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    // =========================
    // LOGGING (AdvantageScope)
    // =========================
    private final DataLog log = DataLogManager.getLog();

    private final DoubleArrayLogEntry odomLog =
        new DoubleArrayLogEntry(log, "/Swerve/OdomPose");

    private final DoubleArrayLogEntry visionLog =
        new DoubleArrayLogEntry(log, "/Swerve/VisionPose");

    private final DoubleLogEntry errorLog =
        new DoubleLogEntry(log, "/Swerve/VisionError");

    // Last pose that passed all gates and was fused (used for error logging)
    private volatile Pose2d lastAcceptedVisionPose  = new Pose2d();
    // Last pose attempted regardless of rejection (used for Field2d ghost)
    private volatile Pose2d lastAttemptedVisionPose = new Pose2d();

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
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        DataLogManager.start();
        SmartDashboard.putData("Field", field);
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
    }

    // =========================
    // AUTO BUILDER (NON-PRO)
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
                driveRequest.withSpeeds(
                    ChassisSpeeds.discretize(speeds, 0.02)
                )
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

        // Update field widget (odometry pose)
        field.setRobotPose(pose);
        // Ghost shows every vision attempt — updates even when rejected
        field.getObject("VisionPose").setPose(lastAttemptedVisionPose);

        odomLog.append(new double[]{
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        });

        // Error is against the last *accepted* vision pose (post-fix)
        double error = pose.getTranslation()
            .getDistance(lastAcceptedVisionPose.getTranslation());

        errorLog.append(error);
    }

    // =========================
    // VISION FUSION
    // =========================
    @Override
    public void addVisionMeasurement(
        Pose2d visionPose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        // Always record the attempt so Field2d ghost stays live
        lastAttemptedVisionPose = visionPose;

        // =========================
        // LATENCY ALIGNMENT
        // =========================
        Optional<Pose2d> odomAtTime = samplePoseAt(timestampSeconds);
        if (odomAtTime.isEmpty()) return;

        Pose2d odomPose = odomAtTime.get();

        // =========================
        // REJECTION (GATING)
        // =========================
        double error = visionPose.getTranslation()
            .getDistance(odomPose.getTranslation());

        if (error > MAX_VISION_ERROR) return;

        double omega = Math.abs(getState().Speeds.omegaRadiansPerSecond);
        if (omega > MAX_ANGULAR_RATE) return;

        // =========================
        // HARD RELOCALIZATION
        // =========================
        if (error > SNAP_ERROR) {
            resetPose(visionPose);
            lastAcceptedVisionPose = visionPose;
            return;
        }

        // =========================
        // DYNAMIC TRUST SCALING
        // =========================
        double scaledXY    = stdDevs.get(0, 0) + error * 0.5;
        double scaledTheta = stdDevs.get(2, 0) + error * 0.3;

        Matrix<N3, N1> scaledStd =
            VecBuilder.fill(scaledXY, scaledXY, scaledTheta);

        // =========================
        // LOG + FUSE
        // FIX: lastVisionPose updated here, after all checks pass
        // =========================
        lastAcceptedVisionPose = visionPose;

        visionLog.append(new double[]{
            visionPose.getX(),
            visionPose.getY(),
            visionPose.getRotation().getRadians()
        });

        super.addVisionMeasurement(
            visionPose,
            Utils.fpgaToCurrentTime(timestampSeconds),
            scaledStd
        );
    }

    // =========================
    // HIGH-CONFIDENCE SNAP
    // Called when vision is trusted enough to override odometry entirely
    // (e.g. 2+ tags, low ambiguity). Still guards against spinning / wild jumps.
    // =========================
    public void snapToVision(Pose2d visionPose, double timestampSeconds) {

        lastAttemptedVisionPose = visionPose;

        double omega = Math.abs(getState().Speeds.omegaRadiansPerSecond);
        if (omega > MAX_ANGULAR_RATE) return;

        Optional<Pose2d> odomAtTime = samplePoseAt(timestampSeconds);
        if (odomAtTime.isEmpty()) return;

        double error = visionPose.getTranslation()
            .getDistance(odomAtTime.get().getTranslation());

        // Still reject completely insane jumps (camera glitch / wrong tag)
        if (error > MAX_VISION_ERROR) return;

        resetPose(visionPose);
        lastAcceptedVisionPose = visionPose;

        visionLog.append(new double[]{
            visionPose.getX(),
            visionPose.getY(),
            visionPose.getRotation().getRadians()
        });
    }

    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }

    // FIX: removed resetPose() override — it was a pure no-op (just called super).
}