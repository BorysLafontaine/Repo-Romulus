package frc.robot.commands;

import custom.ShotCalculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SS_TurretAim;

public class TurretAimAtHub_CMD extends Command {

    // =========================
    // HUB POSITIONS — FE-2026 Rebuilt Welded
    // VelocityCalculator: HUB_X = 182.11" = 4.626m, HUB_Y = 158.84" = 4.035m
    // Red hub X = 651.22 - 182.11 = 469.11" = 11.915m
    // =========================
    private static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
    private static final Translation2d RED_HUB  = new Translation2d(11.915, 4.035);

    // hubForward: direction the hub "faces" into the field.
    // ShotCalculator uses this to detect if the robot is BEHIND the hub (invalid shot).
    private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);
    private static final Translation2d RED_HUB_FORWARD  = new Translation2d(-1, 0);

    // =========================
    // TURRET ENCODER OFFSET
    // Encoder 0 = robot-right. WPILib robot-relative 0° = forward.
    // Offset to convert: robot-forward(0°) → turret(+90°), robot-right(-90°) → turret(0°).
    // ⚠️ If turret aims 90° off, flip sign to -90.
    // =========================
    private static final double TURRET_ENCODER_OFFSET_DEG = 90.0;

    private final SS_TurretAim            turret;
    private final CommandSwerveDrivetrain drivetrain;
    private final ShotCalculator          calculator;

    // Last commanded angle — used in end() to hold position
    private double lastTurretDeg = 0.0;

    public TurretAimAtHub_CMD(SS_TurretAim turret, CommandSwerveDrivetrain drivetrain) {
        this.turret     = turret;
        this.drivetrain = drivetrain;

        // =========================
        // SHOT CALCULATOR CONFIG
        // Used for SOTM velocity-compensated heading only (RPM output is ignored).
        // =========================
        ShotCalculator.Config cfg = new ShotCalculator.Config();
        cfg.launcherOffsetX    = SS_TurretAim.TURRET_OFFSET_X; // -0.1651m (6.5" back)
        cfg.launcherOffsetY    = SS_TurretAim.TURRET_OFFSET_Y; //  0.0
        cfg.minScoringDistance = 0.3;
        cfg.maxScoringDistance = 10.0;
        cfg.minSOTMSpeed       = 0.1;   // < 0.1 m/s → static aim (no velocity comp)
        cfg.maxSOTMSpeed       = 5.0;
        cfg.maxTiltDeg         = 90.0;  // disabled — no tilt sensor wired

        this.calculator = new ShotCalculator(cfg);

        // Linear TOF LUT at ~15 m/s ball speed.
        // Drives SOTM velocity compensation angle, not shooter RPM.
        calculator.loadLUTEntry(0.5,  0, 0.033);
        calculator.loadLUTEntry(1.0,  0, 0.067);
        calculator.loadLUTEntry(2.0,  0, 0.133);
        calculator.loadLUTEntry(3.0,  0, 0.200);
        calculator.loadLUTEntry(4.0,  0, 0.267);
        calculator.loadLUTEntry(5.0,  0, 0.333);
        calculator.loadLUTEntry(6.0,  0, 0.400);
        calculator.loadLUTEntry(8.0,  0, 0.533);
        calculator.loadLUTEntry(10.0, 0, 0.667);

        addRequirements(turret);
    }

    // =========================
    // INITIALIZE
    // Reset the solver warm-start so stale data from before this command ran
    // doesn't corrupt the first cycle (especially after a pose hard-reset).
    // =========================
    @Override
    public void initialize() {
        calculator.resetWarmStart();
    }

    @Override
    public void execute() {

        Pose2d robotPose   = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;

        // Convert robot-relative speeds → field-relative for ShotCalculator
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, robotPose.getRotation()
        );

        boolean isRed    = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub        = isRed ? RED_HUB         : BLUE_HUB;
        Translation2d hubForward = isRed ? RED_HUB_FORWARD : BLUE_HUB_FORWARD;

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            robotPose,
            fieldSpeeds,
            robotSpeeds,
            hub,
            hubForward,
            1.0  // vision confidence — localization handled by Kalman upstream
        );

        ShotCalculator.LaunchParameters result = calculator.calculate(inputs);

        double turretDeg;
        double turretVelDegPerSec;

        if (result.isValid()) {
            // driveAngle: field-relative aim heading, velocity-compensated.
            // Convert to robot-relative, then apply encoder offset.
            double fieldAimDeg = result.driveAngle().getDegrees();
            double headingDeg  = robotPose.getRotation().getDegrees();
            double robotRelDeg = MathUtil.inputModulus(fieldAimDeg - headingDeg, -180.0, 180.0);
            turretDeg = robotRelDeg + TURRET_ENCODER_OFFSET_DEG;

            // Turret angular velocity feedforward:
            //   d/dt(turret) = d/dt(field aim angle) − d/dt(robot heading)
            // driveAngularVelocityRadPerSec = d/dt(field aim angle) from ShotCalculator.
            // fieldSpeeds.omega = d/dt(robot heading).
            double turretVelRadPerSec =
                result.driveAngularVelocityRadPerSec() - fieldSpeeds.omegaRadiansPerSecond;
            turretVelDegPerSec = Math.toDegrees(turretVelRadPerSec);

        } else {
            // Fallback: plain atan2 from turret pivot to hub (no velocity comp).
            // Runs when robot is stationary, out of range, or solver diverged.
            Translation2d pivot = robotPose.getTranslation().plus(
                new Translation2d(SS_TurretAim.TURRET_OFFSET_X, SS_TurretAim.TURRET_OFFSET_Y)
                    .rotateBy(robotPose.getRotation())
            );
            double dx = hub.getX() - pivot.getX();
            double dy = hub.getY() - pivot.getY();
            double fieldAngDeg = Math.toDegrees(Math.atan2(dy, dx));
            double robotRelDeg = MathUtil.inputModulus(
                fieldAngDeg - robotPose.getRotation().getDegrees(), -180.0, 180.0
            );
            turretDeg = robotRelDeg + TURRET_ENCODER_OFFSET_DEG;

            // No velocity compensation available — let PID do all the work
            turretVelDegPerSec = 0.0;
        }

        // Wrap into turret travel range [-180°, +180°]
        turretDeg = MathUtil.inputModulus(
            turretDeg, SS_TurretAim.SOFT_MIN_DEG, SS_TurretAim.SOFT_MAX_DEG
        );

        lastTurretDeg = turretDeg;

        // Use tracking mode (PositionVoltage + velocity FF) for smooth continuous aim
        turret.setAngleDegreesTracking(turretDeg, turretVelDegPerSec);

        // =========================
        // DIAGNOSTICS
        // =========================
        double actualDeg = turret.getAngleDegrees();
        double distM     = result.isValid() ? result.solvedDistanceM() : 0.0;
        double errorDeg  = turretDeg - actualDeg;

        Logger.recordOutput("Turret/TargetDeg",       turretDeg);
        Logger.recordOutput("Turret/ActualDeg",       actualDeg);
        Logger.recordOutput("Turret/ErrorDeg",        errorDeg);
        Logger.recordOutput("Turret/VelFFDegPerSec",  turretVelDegPerSec);
        Logger.recordOutput("Turret/OnTarget",        turret.isOnTarget(turretDeg));
        Logger.recordOutput("Turret/SolverValid",     result.isValid());
        Logger.recordOutput("Turret/Confidence",      result.confidence());
        Logger.recordOutput("Turret/DistM",           distM);
        Logger.recordOutput("Turret/Alliance",        isRed ? "Red" : "Blue");

        SmartDashboard.putNumber ("Turret/TargetDeg",  turretDeg);
        SmartDashboard.putNumber ("Turret/ActualDeg",  actualDeg);
        SmartDashboard.putNumber ("Turret/ErrorDeg",   errorDeg);
        SmartDashboard.putNumber ("Turret/Confidence", result.confidence());
        SmartDashboard.putNumber ("Turret/DistM",      distM);
        SmartDashboard.putBoolean("Turret/OnTarget",   turret.isOnTarget(turretDeg));
        SmartDashboard.putBoolean("Turret/SOTMValid",  result.isValid());
    }

    // =========================
    // END — hold last commanded position (Brake mode keeps motor locked)
    // =========================
    @Override
    public void end(boolean interrupted) {
        turret.setAngleDegrees(lastTurretDeg);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
