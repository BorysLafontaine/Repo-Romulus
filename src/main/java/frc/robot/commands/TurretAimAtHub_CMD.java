package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.SS_TurretAim;

public class TurretAimAtHub_CMD extends Command {

    // =========================
    // HUB POSITIONS (field-relative, meters)
    // ⚠️ Verify against the official 2026 field layout document.
    // =========================
    private static final Translation2d BLUE_HUB = new Translation2d(0.46,  4.11);
    private static final Translation2d RED_HUB  = new Translation2d(16.08, 4.11);

    private final SS_TurretAim     turret;
    private final Supplier<Pose2d> robotPoseSupplier;

    public TurretAimAtHub_CMD(SS_TurretAim turret, Supplier<Pose2d> robotPoseSupplier) {
        this.turret            = turret;
        this.robotPoseSupplier = robotPoseSupplier;
        addRequirements(turret);
    }

    @Override
    public void execute() {

        Pose2d        robotPose = robotPoseSupplier.get();
        Translation2d hub       = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? RED_HUB
            : BLUE_HUB;

        // =========================
        // TURRET PIVOT IN FIELD COORDINATES
        // The offset vector is fixed to the robot frame, so it must be
        // rotated by the robot heading before adding to the field position.
        // =========================
        Translation2d pivotOffset = new Translation2d(
            SS_TurretAim.TURRET_OFFSET_X,
            SS_TurretAim.TURRET_OFFSET_Y
        ).rotateBy(robotPose.getRotation());

        Translation2d turretPivot = robotPose.getTranslation().plus(pivotOffset);

        // =========================
        // ANGLE CALCULATION
        // 1. Field-relative angle from turret pivot to hub
        // 2. Subtract robot heading → robot-relative turret angle
        // 3. Normalize to [-180°, +180°)
        // 4. Map into turret range [-270°, +90°]:
        //    Any value > 90° is outside cable limits → subtract 360°
        //    to bring it into (-270°, -180°], which is within range.
        //    The turret covers exactly 360° so every angle has one valid slot.
        // =========================
        double dx            = hub.getX() - turretPivot.getX();
        double dy            = hub.getY() - turretPivot.getY();
        double fieldAngleRad = Math.atan2(dy, dx);
        double robotRelRad   = fieldAngleRad - robotPose.getRotation().getRadians();

        double turretDeg = Math.toDegrees(MathUtil.angleModulus(robotRelRad));
        if (turretDeg > SS_TurretAim.SOFT_MAX_DEG) turretDeg -= 360.0;

        turret.setAngleDegrees(turretDeg);

        Logger.recordOutput("Turret/Hub/TargetDeg", turretDeg);
        Logger.recordOutput("Turret/Hub/DistanceM", turretPivot.getDistance(hub));
        Logger.recordOutput("Turret/Hub/OnTarget",  turret.isOnTarget(turretDeg));
    }

    // No end() stop — this is the default command.
    // When interrupted by the zero failsafe the motor just holds position.
    // This command resumes automatically when the failsafe button is released.

    @Override
    public boolean isFinished() {
        return false;
    }
}