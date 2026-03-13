package custom;

import custom.VelocityCalculator;

public class ShooterManager {

    private final ShooterTracker  tracker;
    private final VelocityCalculator velocityCalc;

    public ShooterManager() {
        tracker      = new ShooterTracker();
        velocityCalc = new VelocityCalculator();
    }

    /**
     * À appeler dans la boucle principale du robot (periodic).
     * Met à jour la rotation de la tourelle et retourne la vélocité de tir.
     *
     * @param robotX         Position X du robot en pouces.
     * @param robotY         Position Y du robot en pouces.
     * @param robotAngle     Cap du robot en radians.
     * @param batteryVoltage Tension actuelle de la batterie en volts.
     */
    public void update(double robotX, double robotY, double robotAngle, double batteryVoltage) {
        tracker.update(robotX, robotY, robotAngle);
    }

    /**
     * Retourne l'angle auquel positionner la tourelle (en radians).
     * À envoyer directement au contrôleur du moteur de rotation.
     */
    public double getTurretAngle() {
        return tracker.getCurrentRotation();
    }

    /**
     * Retourne la vélocité à envoyer au shooter (compensée en tension).
     */
    public double getShootingVelocity(double robotX, double robotY, double batteryVoltage) {
        return velocityCalc.calculateShootingVelocity(robotX, robotY, batteryVoltage);
    }

    /**
     * Retourne true si la tourelle est alignée avec le hub (tolérance en radians).
     * Utile pour savoir si le robot est prêt à tirer.
     */
    public boolean isReady(double toleranceRad) {
        // La tourelle est "prête" si elle n'a plus besoin de bouger
        // (à compléter selon la logique de ton système de contrôle)
        return Math.abs(tracker.getCurrentRotation()) <= toleranceRad;
    }
}


/* exemple utilisation

ShooterManager shooter = new ShooterManager();

// Dans teleopPeriodic() ou une Command :
shooter.update(robotX, robotY, robotAngle, RobotController.getBatteryVoltage());

if (shooter.isReady(0.05)) {
    // tirer !
    double velocity = shooter.getShootingVelocity(robotX, robotY, RobotController.getBatteryVoltage());
}

*/