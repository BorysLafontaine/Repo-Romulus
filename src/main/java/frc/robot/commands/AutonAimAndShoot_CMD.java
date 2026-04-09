package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_TurretAim;

/**
 * Auton command: aims the turret at the hub and spins the shooter to the
 * distance-based RPM simultaneously. Runs until cancelled by PathPlanner.
 * Transfer is NOT activated — trigger it separately when ready to fire.
 */
public class AutonAimAndShoot_CMD extends Command {

    private static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
    private static final Translation2d RED_HUB  = new Translation2d(11.915, 4.035);

    private static final double TURRET_ENCODER_OFFSET_DEG = -90.0;

    private final SS_TurretAim            turret;
    private final SS_Shooter              shooter;
    private final CommandSwerveDrivetrain drivetrain;

    public AutonAimAndShoot_CMD(
        SS_TurretAim turret,
        SS_Shooter shooter,
        CommandSwerveDrivetrain drivetrain
    ) {
        this.turret     = turret;
        this.shooter    = shooter;
        this.drivetrain = drivetrain;
        addRequirements(turret, shooter);
    }

    @Override
    public void initialize() {
        shooter.resetController();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getOdometryPose();

        boolean isRed     = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub = isRed ? RED_HUB : BLUE_HUB;

        // Turret pivot in field coordinates
        Translation2d pivot = robotPose.getTranslation().plus(
            new Translation2d(SS_TurretAim.TURRET_OFFSET_X, SS_TurretAim.TURRET_OFFSET_Y)
                .rotateBy(robotPose.getRotation())
        );

        double dx = hub.getX() - pivot.getX();
        double dy = hub.getY() - pivot.getY();

        // Field-relative → robot-relative → turret encoder space
        double fieldAngDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotRelDeg = MathUtil.inputModulus(
            fieldAngDeg - robotPose.getRotation().getDegrees(), -180.0, 180.0
        );
        double turretDeg = MathUtil.inputModulus(
            robotRelDeg + TURRET_ENCODER_OFFSET_DEG,
            SS_TurretAim.SOFT_MIN_DEG, SS_TurretAim.SOFT_MAX_DEG
        );
        turret.setAngleDegrees(turretDeg);

        // Distance-based RPM
        double distM = Math.hypot(dx, dy);
        shooter.setRPMFromDistance(distM);

        boolean readyToFire = turret.isOnTarget(turretDeg) && shooter.isAtSpeed();

        Logger.recordOutput("Auton/AimShoot/DistM",       distM);
        Logger.recordOutput("Auton/AimShoot/TurretDeg",   turretDeg);
        Logger.recordOutput("Auton/AimShoot/ReadyToFire", readyToFire);

        SmartDashboard.putBoolean("Auton/ReadyToFire", readyToFire);
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
