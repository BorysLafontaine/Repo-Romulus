package frc.robot.subsystems;

import java.util.Comparator;
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
    // ALLIANCE PERSPECTIVE
    // Blue = driver pushes forward → robot moves toward Red wall (field +X).
    // Red  = driver pushes forward → robot moves toward Blue wall (field -X).
    // ⚠️ This ONLY affects field-centric drive direction, not pose coordinates.
    // =========================
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.k180deg;
    private static final Rotation2d kRedAlliancePerspectiveRotation  = Rotation2d.kZero;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds driveRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    // =========================
    // LOGGING
    // =========================
    private final DataLog log = DataLogManager.getLog();
    private final DoubleArrayLogEntry odomLog   = new DoubleArrayLogEntry(log, "/Swerve/OdomPose");
    private final DoubleArrayLogEntry visionLog = new DoubleArrayLogEntry(log, "/Swerve/VisionPose");

    private Pose2d lastVisionPose = new Pose2d();

    // Odometry pose captured before vision resets — pure wheel odometry, no camera influence
    private Pose2d odomOnlyPose = new Pose2d();

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
        field.getObject("VisionPose").setPose(lastVisionPose);

        odomLog.append(new double[]{
            pose.getX(), pose.getY(), pose.getRotation().getRadians()
        });

        // Publish current pose heading for turret debugging
        SmartDashboard.putNumber("Odometry/X",       pose.getX());
        SmartDashboard.putNumber("Odometry/Y",       pose.getY());
        SmartDashboard.putNumber("Odometry/HeadDeg", pose.getRotation().getDegrees());
    }

    // =========================
    // VISION POSE OVERRIDE
    //
    // Every cycle, pick the best available estimate and hard-reset XY from it.
    // Gyro heading is always preserved — vision heading is ignored.
    //
    // Priority: multi-tag (lowest ambiguity) > single-tag (lowest ambiguity).
    // Falls back to pure odometry when no estimates pass the filter.
    // =========================
    public void addVisionMeasurements(List<VisionEstimate> estimates) {

        boolean hasTags = !estimates.isEmpty();
        SmartDashboard.putBoolean("Vision/TagsVisible", hasTags);

        if (estimates.isEmpty()) return;

        // Prefer multi-tag (unambiguous), fall back to best single-tag
        VisionEstimate best = estimates.stream()
            .filter(e -> e.tagCount() >= 2)
            .min(Comparator.comparingDouble(VisionEstimate::avgAmbiguity))
            .orElseGet(() -> estimates.stream()
                .min(Comparator.comparingDouble(VisionEstimate::avgAmbiguity))
                .orElse(null));

        if (best == null) return;

        // Hard-reset XY from vision, keep gyro heading
        Pose2d resetTo = new Pose2d(best.pose().getTranslation(), getState().Pose.getRotation());
        resetPose(resetTo);
        lastVisionPose = resetTo;

        double dist = best.avgDistanceMeters();
        visionLog.append(new double[]{resetTo.getX(), resetTo.getY(), resetTo.getRotation().getRadians()});

        SmartDashboard.putNumber ("Vision/PoseX",    resetTo.getX());
        SmartDashboard.putNumber ("Vision/PoseY",    resetTo.getY());
        SmartDashboard.putNumber ("Vision/Dist",     dist);
        SmartDashboard.putNumber ("Vision/TagCount", best.tagCount());
        SmartDashboard.putNumber ("Vision/Ambiguity",best.avgAmbiguity());
    }

    // =========================
    // MANUAL HARD RESET (Start button / teleop-enable)
    // Same as the automatic override but callable from a button.
    // =========================
    public boolean hardResetPoseFromVision(VisionEstimate est) {
        Pose2d resetTo = new Pose2d(est.pose().getTranslation(), getState().Pose.getRotation());
        resetPose(resetTo);
        lastVisionPose = resetTo;
        SmartDashboard.putBoolean("Vision/ManualResetTriggered", true);
        return true;
    }

    // =========================
    // ODOMETRY-ONLY POSE (no camera influence)
    // Captured before vision resets the pose each cycle.
    // Use this for distance-based calculations that should not jump with vision.
    // =========================
    public void snapshotOdometryPose(Pose2d pose) {
        odomOnlyPose = pose;
    }

    public Pose2d getOdometryPose() {
        return odomOnlyPose;
    }

    // =========================
    // OVERRIDES
    // =========================
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
