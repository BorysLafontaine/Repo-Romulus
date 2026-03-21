package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
public class SS_Intake extends SubsystemBase {

    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 0);
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    public SS_Intake() {
        ShuffleboardTab tab = Shuffleboard.getTab("Pneumatics");
        tab.add("Double Solenoid", m_doubleSolenoid);
        tab.add("Compressor", m_compressor);
        tab.addDouble("PH Pressure [PSI]", m_compressor::getPressure);
        tab.addDouble("Compressor Current", m_compressor::getCurrent);
        tab.addBoolean("Compressor Active", m_compressor::isEnabled);
        tab.addBoolean("Pressure Switch", m_compressor::getPressureSwitchValue);

        m_compressor.enableDigital();
    }

    public void deploy() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggleDeploy() {
        if (m_doubleSolenoid.get() == DoubleSolenoid.Value.kForward) {
            retract();
        } else {
            deploy();
        }
    }

    @Override
    public void periodic() {}
}
