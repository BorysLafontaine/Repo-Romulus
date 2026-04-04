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
    // HUB POSITIONS — FE-2026 Rebuilt Welded (confirmed by VelocityCalculator.java)
    // VelocityCalculator: HUB_X = 182.11" = 4.626m, HUB_Y = 158.84" = 4.035m
    // Field width: 651.22", so Red hub X = 651.22 - 182.11 = 469.11" = 11.915m
    // =========================
    private static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
    private static final Translation2d RED_HUB  = new Translation2d(11.915, 4.035);

    // hubForward = direction the hub "faces" (away from its wall, into the field).
    // ShotCalculator uses this to detect if the robot is BEHIND the hub.
    // Blue hub is at X=4.626 near the blue wall → faces toward +X (red side).
    // Red hub is at X=11.915 near the red wall → faces toward -X (blue side).
    private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);
    private static final Translation2d RED_HUB_FORWARD  = new Translation2d(-1, 0);

    private final SS_TurretAim            turret;
    private final CommandSwerveDrivetrain drivetrain;
    private final ShotCalculator          calculator;

    public TurretAimAtHub_CMD(SS_TurretAim turret, CommandSwerveDrivetrain drivetrain) {
        this.turret     = turret;
        this.drivetrain = drivetrain;

        // =========================
        // SHOT CALCULATOR CONFIG
        // Only used for SOTM velocity compensation — RPM output is ignored here.
        // Turret pivot position = launcherOffset (from SS_TurretAim constants).
        // =========================
        ShotCalculator.Config cfg = new ShotCalculator.Config();
        cfg.launcherOffsetX    = SS_TurretAim.TURRET_OFFSET_X; // -0.1651m (6.5" back)
        cfg.launcherOffsetY    = SS_TurretAim.TURRET_OFFSET_Y; // 0.0
        cfg.minScoringDistance = 0.3;
        cfg.maxScoringDistance = 10.0;
        cfg.minSOTMSpeed       = 0.1;   // below 0.1 m/s → static aim (no velocity comp)
        cfg.maxSOTMSpeed       = 5.0;
        cfg.maxTiltDeg         = 90.0;  // disable tilt gate — no tilt sensor wired

        this.calculator = new ShotCalculator(cfg);

        // Pre-load a linear TOF approximation (assumes ~15 m/s ball speed).
        // ShotCalculator needs a non-empty TOF LUT to interpolate from.
        // These values drive the SOTM velocity compensation angle, not shooter RPM.
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

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;

        // Convert robot-relative chassis speeds → field-relative for ShotCalculator
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, robotPose.getRotation()
        );

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub        = isRed ? RED_HUB         : BLUE_HUB;
        Translation2d hubForward = isRed ? RED_HUB_FORWARD : BLUE_HUB_FORWARD;

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            robotPose,
            fieldSpeeds,
            robotSpeeds,
            hub,
            hubForward,
            1.0   // vision confidence (full trust — localization is handled upstream)
        );

        ShotCalculator.LaunchParameters result = calculator.calculate(inputs);

        double turretDeg;

        if (result.isValid()) {
            // driveAngle = field-relative heading to aim at (velocity-compensated).
            // Subtract robot heading to get robot-relative turret angle.
            double fieldAimDeg = result.driveAngle().getDegrees();
            double headingDeg  = robotPose.getRotation().getDegrees();
            double robotRelDeg = MathUtil.inputModulus(fieldAimDeg - headingDeg, -180.0, 180.0);

            turretDeg = robotRelDeg;
        } else {
            // Fallback: plain atan2 from turret pivot to hub center (no velocity comp).
            // Runs when robot is stationary, out of SOTM range, or ShotCalculator diverged.
            Translation2d pivot = robotPose.getTranslation().plus(
                new Translation2d(SS_TurretAim.TURRET_OFFSET_X, SS_TurretAim.TURRET_OFFSET_Y)
                    .rotateBy(robotPose.getRotation())
            );
            double dx = hub.getX() - pivot.getX();
            double dy = hub.getY() - pivot.getY();
            double fieldAngDeg = Math.toDegrees(Math.atan2(dy, dx));
            turretDeg = MathUtil.inputModulus(
                fieldAngDeg - robotPose.getRotation().getDegrees(), -180.0, 180.0
            );
        }

        // Wrap into turret travel range [SOFT_MIN_DEG, SOFT_MAX_DEG] = [-90°, 280°]
        if (turretDeg > SS_TurretAim.SOFT_MAX_DEG) turretDeg -= 360.0;
        if (turretDeg < SS_TurretAim.SOFT_MIN_DEG) turretDeg += 360.0;

        turret.setAngleDegrees(turretDeg);

        // --- Diagnostics ---
        double distM = result.isValid() ? result.solvedDistanceM() : 0.0;
        Logger.recordOutput("Turret/TargetDeg",    turretDeg);
        Logger.recordOutput("Turret/ActualDeg",    turret.getAngleDegrees());
        Logger.recordOutput("Turret/OnTarget",     turret.isOnTarget(turretDeg));
        Logger.recordOutput("Turret/SolverValid",  result.isValid());
        Logger.recordOutput("Turret/Confidence",   result.confidence());
        Logger.recordOutput("Turret/DistM",        distM);
        Logger.recordOutput("Turret/Alliance",     isRed ? "Red" : "Blue");

        SmartDashboard.putNumber("Turret/TargetDeg",  turretDeg);
        SmartDashboard.putNumber("Turret/ActualDeg",  turret.getAngleDegrees());
        SmartDashboard.putNumber("Turret/Confidence", result.confidence());
        SmartDashboard.putNumber("Turret/DistM",      distM);
        SmartDashboard.putBoolean("Turret/OnTarget",  turret.isOnTarget(turretDeg));
        SmartDashboard.putBoolean("Turret/SOTMValid", result.isValid());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
