package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class SS_Intake extends SubsystemBase {

    private final Solenoid m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    public SS_Intake() {
        ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
        tab.add("Single Solenoid", m_solenoid);
        tab.add("Double Solenoid", m_doubleSolenoid);
        tab.add("Compressor", m_compressor);
        tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
        tab.addDouble("Compressor Current", m_compressor::getCurrent);
        tab.addBoolean("Compressor Active", m_compressor::isEnabled);
        tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);

        m_compressor.enableAnalog(70, 120);
    }

    public void toggle_Solenoid(){
        
    }

    public void setSolenoid(boolean extended) {
        m_solenoid.set(extended);
    }

    public void deploy() {
        m_doubleSolenoid.set(Value.kForward);
    }

    public void retract() {
        m_doubleSolenoid.set(Value.kReverse);
    }

    public void toggleDeploy() {
        if (m_doubleSolenoid.get() == Value.kForward) {
            retract();
        } else {
            deploy();
        }
    }

    public void toggleCompressor() {
        if (m_compressor.isEnabled()) {
            m_compressor.disable();
        } else {
            m_compressor.enableAnalog(70, 120);
        }
    }

    @Override
    public void periodic() {}
}

    
    


        // Single solenoid — maintenu tant que bouton enfoncé
        m_intake.setSolenoid(m_stick.getRawButton(kSolenoidButton));
        // Double solenoid — deploy ou retract
        if (m_stick.getRawButtonPressed(kDeployButton)) {
            m_intake.deploy();
        } else if (m_stick.getRawButtonPressed(kRetractButton)) {
            m_intake.retract();
        }
        // Toggle compressor
        if (m_stick.getRawButtonPressed(kCompressorButton)) {
            m_intake.toggleCompressor();
        }