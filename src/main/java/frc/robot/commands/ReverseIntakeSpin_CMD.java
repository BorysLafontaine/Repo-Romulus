// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_IntakeMotors;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseIntakeSpin_CMD extends Command {
  /** Creates a new IndexerSpin_CMD. */
  private final SS_IntakeMotors mIRollers;
  public ReverseIntakeSpin_CMD(SS_IntakeMotors pIRoller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIRollers = pIRoller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIRollers.reverseSpinIRoller();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIRollers.stopIRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
