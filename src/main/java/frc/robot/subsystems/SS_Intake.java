// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Intake extends SubsystemBase {

  private final DoubleSolenoid m_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1, 0);
  // État du système 
  private Value deployedState = m_solenoid.get();

  public void deploy_retract() {

        if (deployedState == Value.kForward) {
            retract();
        } else if(deployedState == Value.kReverse){
            deploy();
        }
    }

  // ─── Déploiement ─────────────────────────────────────────────────────────
  private void deploy() {

      m_solenoid.set(DoubleSolenoid.Value.kForward);
      System.out.println(" intake DEPLOYED");
  }

  // ─── Rétractation ────────────────────────────────────────────────────────
  private void retract() {
      m_solenoid.set(DoubleSolenoid.Value.kReverse);
      System.out.println(" intake RETRACTED");
  }
  public void update(){
    deployedState = m_solenoid.get();
    SmartDashboard.putString("solenoid state" , deployedState.toString());
  }


  @Override
  public void periodic() {
    update();
  }
}
