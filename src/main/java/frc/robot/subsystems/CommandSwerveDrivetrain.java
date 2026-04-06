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
    // VISION MEASUREMENT FUSION
    //
    // Uses the drivetrain's built-in Kalman filter (addVisionMeasurement).
    // XY standard deviation scales with distance² — farther tags trusted less.
    // Rotation is never trusted from vision (huge stddev) — gyro handles heading.
    //
    // This is called from the telemetry callback (odometry thread), so it runs
    // at the odometry frequency (250 Hz on CAN FD) — vision frames are typically
    // 20–50 Hz so most calls are no-ops (estimator returns empty Optional).
    // =========================
    public void addVisionMeasurements(List<VisionEstimate> estimates) {

        boolean hasTags = !estimates.isEmpty();
        SmartDashboard.putBoolean("Vision/TagsVisible", hasTags);

        for (VisionEstimate est : estimates) {

            double dist = Math.max(est.avgDistanceMeters(), 0.1);

            // Multi-tag PNP is geometrically unambiguous → trust it significantly more.
            // Single-tag: quadratic growth with distance (perspective error).
            // Capped at 2.0m — beyond that, don't let a noisy single-tag corrupt odometry.
            double xyStdDev;
            if (est.tagCount() >= 2) {
                xyStdDev = 0.01 * dist * dist; // tight: 0.04m at 2m, 0.09m at 3m
            } else {
                xyStdDev = 0.05 * dist * dist; // loose: 0.20m at 2m, 0.45m at 3m
            }
            xyStdDev = Math.min(xyStdDev, 2.0);

            // Never correct heading from vision — trust IMU/gyro instead
            double rotStdDev = 9999.0;

            addVisionMeasurement(
                est.pose(),
                est.timestampSeconds(),
                VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
            );

            lastVisionPose = est.pose();

            visionLog.append(new double[]{
                est.pose().getX(), est.pose().getY(), est.pose().getRotation().getRadians()
            });

            SmartDashboard.putNumber("Vision/PoseX",    est.pose().getX());
            SmartDashboard.putNumber("Vision/PoseY",    est.pose().getY());
            SmartDashboard.putNumber("Vision/Dist",     dist);
            SmartDashboard.putNumber("Vision/XYStdDev", xyStdDev);
        }
    }

    // =========================
    // MANUAL HARD RESET (Start button / teleop-enable)
    // Instantly snaps odometry to vision pose — use when stationary with clear tag view.
    // =========================
    public boolean hardResetPoseFromVision(VisionEstimate est) {
        // Keep gyro heading, override only XY from vision
        Pose2d resetTo = new Pose2d(est.pose().getTranslation(), getState().Pose.getRotation());
        resetPose(resetTo);
        lastVisionPose = resetTo;
        SmartDashboard.putBoolean("Vision/ManualResetTriggered", true);
        return true;
    }

    // =========================
    // OVERRIDES
    // =========================
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
