package frc.robot.commands;

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
 * Spins the shooter to the RPM from the distance-based LUT in SS_Shooter.
 * Distance is computed each cycle from the robot's odometry pose to the hub.
 */
public class ShooterSpin_CMD extends Command {

    private static final Translation2d BLUE_HUB = new Translation2d(4.626, 4.035);
    private static final Translation2d RED_HUB  = new Translation2d(11.915, 4.035);

    private final SS_Shooter              shooter;
    private final CommandSwerveDrivetrain drivetrain;

    public ShooterSpin_CMD(SS_Shooter shooter, CommandSwerveDrivetrain drivetrain) {
        this.shooter    = shooter;
        this.drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetController();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getOdometryPose();

        boolean isRed        = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d hub    = isRed ? RED_HUB : BLUE_HUB;

        // Distance from turret pivot to hub
        Translation2d pivot = robotPose.getTranslation().plus(
            new Translation2d(SS_TurretAim.TURRET_OFFSET_X, SS_TurretAim.TURRET_OFFSET_Y)
                .rotateBy(robotPose.getRotation())
        );
        double distM = pivot.getDistance(hub);

        shooter.setRPMFromDistance(distM);

        double targetRPM = shooter.getTargetRPM();

        Logger.recordOutput("Shooter/DistM",     distM);
        Logger.recordOutput("Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/AtSpeed",   shooter.isAtSpeed());

        SmartDashboard.putNumber ("Shooter/DistM",    distM);
        SmartDashboard.putNumber ("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed",  shooter.isAtSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public boolean isAtSpeed() {
        return shooter.isAtSpeed();
    }
}
