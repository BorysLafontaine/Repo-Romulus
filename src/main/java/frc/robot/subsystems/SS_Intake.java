// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.Watchable;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Intake extends SubsystemBase {

  private final Solenoid m_solenoid =             new Solenoid(PneumaticsModuleType.REVPH, 0);
  private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
  private final Compressor m_compressor =         new Compressor(PneumaticsModuleType.REVPH);
  // État du système 
  private Value deployedState;

  static final int kSolenoidButton = 1;
  static final int kDoubleSolenoidForwardButton = 2;
  static final int kDoubleSolenoidReverseButton = 3;
  static final int kCompressorButton = 4;
  
  /** create a new ss_intake */
  public SS_Intake() {
     // Publish elements to shuffleboard.
    ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
    tab.add("Single Solenoid", m_solenoid);
    tab.add("Double Solenoid", m_doubleSolenoid);
    tab.add("Compressor", m_compressor);
    // Also publish some raw data
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
    // Get compressor current draw.
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    // Get whether the compressor is active.
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    // Get the digital pressure switch connected to the PCM/PH.
    // The switch is open when the pressure is over ~120 PSI.
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
  }

  public void deploy_retract() {
       
    }

  // ─── Déploiement ─────────────────────────────────────────────────────────
  private void deploy() {

   
  }

  // ─── Rétractation ────────────────────────────────────────────────────────
  private void retract() {
   
  }
  public void update(){

  }


  @Override
  public void periodic() {
    update();
  }
}



  // Solenoid corresponds to a single solenoid.
  // In this case, it's connected to channel 0 of a PH with the default CAN ID.
  

  // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.


  // Compressor connected to a PH with a default CAN ID (1)
  

  

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Publish elements to shuffleboard.
    ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
    tab.add("Single Solenoid", m_solenoid);
    tab.add("Double Solenoid", m_doubleSolenoid);
    tab.add("Compressor", m_compressor);

    // Also publish some raw data
    // Get the pressure (in PSI) from the analog sensor connected to the PH.
    // This function is supported only on the PH!
    // On a PCM, this function will return 0.
    tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
    // Get compressor current draw.
    tab.addDouble("Compressor Current", m_compressor::getCurrent);
    // Get whether the compressor is active.
    tab.addBoolean("Compressor Active", m_compressor::isEnabled);
    // Get the digital pressure switch connected to the PCM/PH.
    // The switch is open when the pressure is over ~120 PSI.
    tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);
  }

  @SuppressWarnings("PMD.UnconditionalIfStatement")
  @Override
  public void teleopPeriodic() {
    /*
     * The output of GetRawButton is true/false depending on whether
     * the button is pressed; Set takes a boolean for whether
     * to retract the solenoid (false) or extend it (true).
     */
    m_solenoid.set(m_stick.getRawButton(kSolenoidButton));

    /*
     * GetRawButtonPressed will only return true once per press.
     * If a button is pressed, set the solenoid to the respective channel.
     */
    if (m_stick.getRawButtonPressed(kDoubleSolenoidForwardButton)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    } else if (m_stick.getRawButtonPressed(kDoubleSolenoidReverseButton)) {
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // On button press, toggle the compressor.
    if (m_stick.getRawButtonPressed(kCompressorButton)) {
      boolean isCompressorEnabled = m_compressor.isEnabled();
      if (isCompressorEnabled) {
        m_compressor.disable();
      } else {
        if (false) {
          m_compressor.enableDigital();
        }
        if (true) {
          m_compressor.enableAnalog(70, 120);
        }
        if (false) {
          m_compressor.enableHybrid(70, 120);
        }
      }
    }
