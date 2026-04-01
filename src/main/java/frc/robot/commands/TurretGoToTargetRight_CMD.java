package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_TurretFixed;

public class TurretGoToTargetRight_CMD extends Command {

  private final SS_TurretFixed mTurretFixed;

  // Constructeur
  public TurretGoToTargetRight_CMD(SS_TurretFixed pTurretFixed) {
    mTurretFixed = pTurretFixed;
  }
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mTurretFixed.turretGoToTargetRight();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTurretFixed.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

