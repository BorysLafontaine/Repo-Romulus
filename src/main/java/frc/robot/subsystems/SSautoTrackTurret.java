// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import custom.ShooterTracker;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SSautoTrackTurret extends SubsystemBase {

  // Position du hub en pouces (constante du terrain)
  private static final double HUB_X = 182.11;
  private static final double HUB_Y = 158.84;
  
  // Limite de rotation maximale : ±π radians (±180°) pour éviter d'enrouler les câbles
  private static final double MAX_ROTATION = Math.PI/2;

  /**
   * Décalage de la tourelle par rapport au centre du robot (en pouces).
   * Si la tourelle est centrée sur le robot, laisser à (0, 0).
   * Exemple : tourelle décalée de 6.5 pouces vers l'arrière (axe X du robot).
   */
  private static final double TURRET_OFFSET_X = -6.5;
  private static final double TURRET_OFFSET_Y = 0.0;

  // Rotation actuelle de la tourelle en radians, maintenue dans [-2π, 2π]
  private double currentRotation = 0.0;
  private final TalonFX mTurretRotationMotor;
  private final ShooterTracker mShooterTracker = new ShooterTracker();

  /** Creates a new SSautoTrackTurret. */
  public SSautoTrackTurret() {

    mShooterTracker.update(0, 0, 0);
    mTurretRotationMotor = new TalonFX(41);
  }

  @Override
  public void periodic() {
    
    mTurretRotationMotor.setPosition(null);
  }

     // ── Mise à jour (à appeler à chaque cycle du robot) ──────────

    /**
     * Met à jour la rotation de la tourelle pour viser le hub.
     * @param robotX     Position X du robot (centre du robot) en pouces.
     * @param robotY     Position Y du robot (centre du robot) en pouces.
     * @param robotAngle Angle actuel du robot en radians (cap du robot sur le terrain),
     *                   nécessaire pour convertir le décalage de la tourelle en coordonnées terrain.
     */
    public void update(double robotX, double robotY, double robotAngle) {
        
        // ── Si le robot dépasse le hub en X (+ 1 pouce), on ne traque pas ──
        if (robotX > HUB_X + 0.5) {           
            System.out.println("too far hub — tracking deactivated");
            return;
        }

        // ── Calcul de la position réelle de la tourelle sur le terrain ──
        // Le décalage est exprimé dans le repère du robot (avant/côté),
        // il faut le faire pivoter selon l'angle du robot pour l'exprimer en coordonnées terrain.
        double turretX = robotX + TURRET_OFFSET_X * Math.cos(robotAngle)
                                - TURRET_OFFSET_Y * Math.sin(robotAngle);
        double turretY = robotY + TURRET_OFFSET_X * Math.sin(robotAngle)
                                + TURRET_OFFSET_Y * Math.cos(robotAngle);

        // ── Calcul de l'angle cible depuis la tourelle (et non le centre du robot) ──
        double target = getAngleToHub(turretX, turretY);
        double delta  = shortestDelta(currentRotation, target);

        double projectedForward  = currentRotation + delta;
        double projectedBackward = currentRotation - delta;

        boolean forwardSafe  = withinBounds(projectedForward);
        boolean backwardSafe = withinBounds(projectedBackward);

        if (forwardSafe && backwardSafe) {
            // Les deux directions sont sûres → prendre le chemin le plus court
            currentRotation = projectedForward;
        } else if (forwardSafe) {
            // Seule la direction avant reste dans les limites
            currentRotation = projectedForward;
        } else if (backwardSafe) {
            // Seule la direction arrière reste dans les limites
            currentRotation = projectedBackward;
        } else {
            // Les deux directions dépassent la limite → aller vers celle qui reste la plus proche de 0
            currentRotation = (Math.abs(projectedForward) < Math.abs(projectedBackward))
                    ? Math.max(-MAX_ROTATION, Math.min(MAX_ROTATION, projectedForward))
                    : Math.max(-MAX_ROTATION, Math.min(MAX_ROTATION, projectedBackward));
        }

        System.out.printf(
            "Robot(" + robotX + ", " + robotY + " ) | Tourelle(" + turretX + ", " + turretY + ") | Cible: " + target + "rad | Rotation: " + currentRotation + " rad\n"
             
        );
    }

    // ── Getter ───────────────────────────────────────────────────

    /** Retourne la rotation actuelle de la tourelle en radians. */
    public double getCurrentRotation() { return currentRotation; }

    // ── Helpers ──────────────────────────────────────────────────

    /** Calcule l'angle depuis une position donnée vers le hub, en radians. */
    private double getAngleToHub(double x, double y) {
        return Math.atan2(HUB_Y - y, HUB_X - x);
    }

    /**
     * Calcule le delta angulaire le plus court entre la rotation courante et la cible.
     * Résultat dans [-π, π].
     */
    private double shortestDelta(double current, double target) {
    double delta = target - current;
    return delta - 2 * Math.PI * Math.round(delta / (2 * Math.PI));
    }

    /** Retourne true si la rotation est dans les limites [-π, π]. */
    private boolean withinBounds(double rotation) {
        return Math.abs(rotation) <= MAX_ROTATION;
    }
}
