package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_TurretAim;

/**
 * Disables the turret by stopping the motor and preventing any movement.
 * While this command is active it holds the requirement, which interrupts the
 * default TurretAimAtHub_CMD. When this command is cancelled (toggle off),
 * the default command resumes automatically.
 *
 * Usage in RobotContainer:
 *   joystick.someButton().toggleOnTrue(new TurretDisable_CMD(mTurretAim));
 */
public class TurretDisable_CMD extends Command {

    private final SS_TurretAim turret;

    public TurretDisable_CMD(SS_TurretAim turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.stop();
    }

    @Override
    public void execute() {
        // Keep calling stop so the motor doesn't drift back to a previous setpoint
        turret.stop();
    }

    @Override
    public void end(boolean interrupted) {
        // Do nothing — releasing the requirement lets the default aim command resume
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
