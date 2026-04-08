package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
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

    // =========================
    // TURRET ENCODER OFFSET
    // Encoder 0 = robot-right. WPILib robot-relative 0° = forward.
    // -90° converts: robot-forward(0°) → turret(-90°), robot-right(−90°) → turret(0°).
    // ⚠️ If turret aims 180° off, flip sign to +90.
    // =========================
    private static final double TURRET_ENCODER_OFFSET_DEG = -90.0;

    private final SS_TurretAim            turret;
    private final CommandSwerveDrivetrain drivetrain;

    public TurretAimAtHub_CMD(SS_TurretAim turret, CommandSwerveDrivetrain drivetrain) {
        this.turret     = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getOdometryPose();

        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub = isRed ? RED_HUB : BLUE_HUB;

        // Compute angle from turret pivot to hub center
        Translation2d pivot = robotPose.getTranslation().plus(
            new Translation2d(SS_TurretAim.TURRET_OFFSET_X, SS_TurretAim.TURRET_OFFSET_Y)
                .rotateBy(robotPose.getRotation())
        );

        double dx = hub.getX() - pivot.getX();
        double dy = hub.getY() - pivot.getY();

        // Field-relative angle → robot-relative → turret encoder space
        double fieldAngDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotRelDeg = MathUtil.inputModulus(
            fieldAngDeg - robotPose.getRotation().getDegrees(), -180.0, 180.0
        );
        double turretDeg = MathUtil.inputModulus(
            robotRelDeg + TURRET_ENCODER_OFFSET_DEG,
            SS_TurretAim.SOFT_MIN_DEG, SS_TurretAim.SOFT_MAX_DEG
        );

        turret.setAngleDegrees(turretDeg);

        // =========================
        // DIAGNOSTICS
        // =========================
        double distM = Math.hypot(dx, dy);

        Logger.recordOutput("Turret/DistM",    distM);
        Logger.recordOutput("Turret/Alliance", isRed ? "Red" : "Blue");

        SmartDashboard.putNumber("Turret/SOTMDistM", distM);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
