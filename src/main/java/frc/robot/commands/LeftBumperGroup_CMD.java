package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.SS_IntakeMotors;
import frc.robot.subsystems.SS_Rollers;
import frc.robot.subsystems.SS_Transfer;

public class LeftBumperGroup_CMD extends ParallelCommandGroup {

    public LeftBumperGroup_CMD(
        SS_IntakeMotors intakeMotors,
        SS_Rollers rollers,
        SS_Transfer transfer
    ) {
        addCommands(
            new IntakeSpin_CMD(intakeMotors),
            new RollerSpin_CMD(rollers),
            new Transfer_CMD(transfer)
        );
    }
}