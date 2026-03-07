package custom;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class deploy_intake extends SubsystemBase{

    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 1,0);

    // ─── État du système ─────────────────────────────────────────────────────
    private boolean isDeployed = false;

    // 
    public Command deploy_retract_button() {
        return runOnce(() -> {
            if (isDeployed) {
                retract();
            } else {
                deploy();
            }
        });
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

    // ─── Accesseurs ──────────────────────────────────────────────────────────
    public boolean isDeployed(){ return isDeployed; }
}