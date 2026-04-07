package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SS_TurretAim;

/**
 * Zeroes the turret encoder at the current physical position — does NOT move the motor.
 * Use this when the turret is manually positioned at your known zero (right side of robot)
 * and you want to re-home the encoder without driving anywhere.
 *
 * Finishes in one cycle. The default TurretAimAtHub_CMD resumes immediately after.
 *
 * Usage in RobotContainer:
 *   joystick.povUp().onTrue(new TurretResetPose_CMD(mTurretAim));
 */
public class TurretResetPose_CMD extends Command {

    private final SS_TurretAim turret;

    public TurretResetPose_CMD(SS_TurretAim turret) {
        this.turret = turret;
        // Requires turret — interrupts TurretAimAtHub_CMD if it is running.
        // Since there is no default command, aim stays off after the reset.
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.resetEncoder();
        SmartDashboard.putBoolean("Turret/EncoderReset", true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Turret/EncoderReset", false);
    }

    @Override
    public boolean isFinished() {
        return true; // instant — done after initialize()
    }
}
