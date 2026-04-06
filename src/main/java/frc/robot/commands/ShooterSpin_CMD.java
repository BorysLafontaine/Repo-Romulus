package frc.robot.commands;

import custom.ShotCalculator;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_TurretAim;

/**
 * SOTM-aware shooter command.
 *
 * Uses ShotCalculator to compute the required RPM at the robot's current distance
 * from the hub, accounting for robot velocity (shoot-on-the-move). Falls back to
 * a fixed mid-range RPM when the solver is out of range or the robot is stationary.
 *
 * Hub coords and launcher offset must match TurretAimAtHub_CMD exactly.
 * The RPM LUT (distance → RPM) must be calibrated on the physical robot.
 */
public class ShooterSpin_CMD extends Command {

    // =========================
    // HUB POSITIONS — must stay in sync with TurretAimAtHub_CMD
    // =========================
    private static final Translation2d BLUE_HUB         = new Translation2d(4.626, 4.035);
    private static final Translation2d RED_HUB           = new Translation2d(11.915, 4.035);
    private static final Translation2d BLUE_HUB_FORWARD = new Translation2d(1, 0);
    private static final Translation2d RED_HUB_FORWARD  = new Translation2d(-1, 0);

    // =========================
    // FALLBACK RPM
    // Used when the solver is invalid (out of range, behind hub, etc.).
    // Set to your mid-range tuned speed.
    // =========================
    private static final double FALLBACK_RPM = 2375.0;

    private final SS_Shooter              shooter;
    private final CommandSwerveDrivetrain drivetrain;
    private final ShotCalculator          calculator;

    public ShooterSpin_CMD(SS_Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        this.shooter    = shooter;
        this.drivetrain = drivetrain;

        ShotCalculator.Config cfg = new ShotCalculator.Config();
        cfg.launcherOffsetX    = SS_TurretAim.TURRET_OFFSET_X; // -0.1651m
        cfg.launcherOffsetY    = SS_TurretAim.TURRET_OFFSET_Y; //  0.0
        cfg.minScoringDistance = 0.3;
        cfg.maxScoringDistance = 10.0;
        cfg.minSOTMSpeed       = 0.1;
        cfg.maxSOTMSpeed       = 5.0;
        cfg.maxTiltDeg         = 90.0; // disabled — no tilt sensor

        this.calculator = new ShotCalculator(cfg);

        // =========================
        // SHOOTER RPM LUT
        // ⚠️ Values are ESTIMATED from the three tuned presets:
        //   1.5m → 1500 RPM (closeSpinShooter)
        //   3.0m → 2375 RPM (spinShooter)
        //   5.0m → 3075 RPM (farSpinShooter)
        // Interpolated entries between anchors. Calibrate by shooting from known distances.
        // TOF column uses ~15 m/s ball speed (same as turret command).
        // =========================
        calculator.loadLUTEntry(0.5,  1200, 0.033);
        calculator.loadLUTEntry(1.0,  1400, 0.067);
        calculator.loadLUTEntry(1.5,  1500, 0.100); // ← tuned anchor
        calculator.loadLUTEntry(2.0,  1900, 0.133);
        calculator.loadLUTEntry(2.5,  2175, 0.167);
        calculator.loadLUTEntry(3.0,  2375, 0.200); // ← tuned anchor
        calculator.loadLUTEntry(4.0,  2750, 0.267);
        calculator.loadLUTEntry(5.0,  3075, 0.333); // ← tuned anchor
        calculator.loadLUTEntry(6.5,  3300, 0.433);
        calculator.loadLUTEntry(8.0,  3600, 0.533);
        calculator.loadLUTEntry(10.0, 3900, 0.667);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Reset warm-start so stale TOF from a previous run doesn't corrupt first cycle
        calculator.resetWarmStart();
    }

    @Override
    public void execute() {
        Pose2d robotPose    = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds, robotPose.getRotation()
        );

        boolean isRed    = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub        = isRed ? RED_HUB         : BLUE_HUB;
        Translation2d hubForward = isRed ? RED_HUB_FORWARD : BLUE_HUB_FORWARD;

        ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
            robotPose, fieldSpeeds, robotSpeeds, hub, hubForward, 1.0
        );

        ShotCalculator.LaunchParameters result = calculator.calculate(inputs);

        double targetRPM;
        if (result.isValid()) {
            targetRPM = result.rpm();
        } else {
            // Out of range or behind hub — spin at mid-range preset
            targetRPM = FALLBACK_RPM;
        }

        shooter.setTargetRPM(targetRPM);

        // =========================
        // DIAGNOSTICS
        // =========================
        double distM = result.isValid() ? result.solvedDistanceM() : 0.0;

        Logger.recordOutput("Shooter/SOTM/Valid",      result.isValid());
        Logger.recordOutput("Shooter/SOTM/TargetRPM",  targetRPM);
        Logger.recordOutput("Shooter/SOTM/DistM",      distM);
        Logger.recordOutput("Shooter/SOTM/Confidence", result.confidence());
        Logger.recordOutput("Shooter/AtSpeed",         shooter.isAtSpeed());

        SmartDashboard.putBoolean("Shooter/SOTMValid",  result.isValid());
        SmartDashboard.putNumber ("Shooter/SOTMDistM",  distM);
        SmartDashboard.putNumber ("Shooter/SOTMTargRPM", targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** Expose readiness for downstream commands (transfer, rollers). */
    public boolean isAtSpeed() {
        return shooter.isAtSpeed();
    }
}
