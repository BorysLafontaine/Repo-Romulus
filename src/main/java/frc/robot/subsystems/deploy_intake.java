// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class deploy_intake extends SubsystemBase {

  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1, 0);
  // État du système 
  private boolean isDeployed = false;

  public void deploy_retract() {
        if (isDeployed) {
            retract();
        } else {
            deploy();
        }
    }

  // ─── Déploiement ─────────────────────────────────────────────────────────
  private void deploy() {
      isDeployed = true;
      m_solenoid.set(DoubleSolenoid.Value.kForward);
      System.out.println(" intake DEPLOYED");
  }

  // ─── Rétractation ────────────────────────────────────────────────────────
  private void retract() {
      isDeployed = false;
      m_solenoid.set(DoubleSolenoid.Value.kReverse);
      System.out.println(" intake RETRACTED");
  }

  public boolean isDeployed(){ return isDeployed; }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
