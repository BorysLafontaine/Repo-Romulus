package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Intake;

public class toggle_intake_CMD extends Command {

    private final SS_Intake m_intake;

    public toggle_intake_CMD(SS_Intake pIntake) {
        m_intake = pIntake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.toggleDeploy();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
