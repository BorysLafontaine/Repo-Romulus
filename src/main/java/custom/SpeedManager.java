package custom;
/**
 * SpeedManager - Gère les vitesses des rollers à partir de proportions de gear.
 * Chaque tableau représente un mode/preset de vitesse.
 * Les valeurs sont des proportions (0.0 à 1.0) appliquées à la vitesse maximale.
 */
public class SpeedManager {

    // ──────────────────────────────────────────────
    //  Tableaux de proportions (gear ratios)
    //  Format : { intakeSpeed, feederSpeed }
    // ──────────────────────────────────────────────

    /** Vitesses pour l'intake uniquement (collecte de balles) */
    public static final double[] INTAKE_ONLY   = { 0.6, 0.0 };

    /** Vitesses pour le feed uniquement (placement dans le réservoir) */
    public static final double[] FEED_ONLY     = { 0.0, 0.7 };

    /** Vitesses pour intake + feed simultanés */
    public static final double[] INTAKE_FEED   = { 0.6, 0.7 };

    /** Vitesses rapides (ex: tir rapide) */
    public static final double[] FAST          = { 1.0, 1.0 };

    /** Vitesses lentes (ex: ajustement précis) */
    public static final double[] SLOW          = { 0.3, 0.3 };

    /** Vitesses en reverse (ex: éjection de balle) */
    public static final double[] REVERSE       = { -0.5, -0.5 };

    /** Vitesses nulles (arrêt complet) */
    public static final double[] STOP          = { 0.0, 0.0 };

    // ──────────────────────────────────────────────
    //  Vitesse maximale physique du moteur (RPM ou %)
    // ──────────────────────────────────────────────
    private static final double MAX_SPEED = 1.0; // 1.0 = 100% du moteur

    /**
     * Calcule la vitesse réelle de l'intake à partir d'un tableau de proportions.
     *
     * @param gearRatios tableau de proportions { intakeRatio, feederRatio }
     * @return vitesse calculée pour l'intake
     */
    public static double getIntakeSpeed(double[] gearRatios) {
        return gearRatios[0] * MAX_SPEED;
    }

    /**
     * Calcule la vitesse réelle du feeder à partir d'un tableau de proportions.
     *
     * @param gearRatios tableau de proportions { intakeRatio, feederRatio }
     * @return vitesse calculée pour le feeder
     */
    public static double getFeederSpeed(double[] gearRatios) {
        return gearRatios[1] * MAX_SPEED;
    }

    /**
     * Retourne les deux vitesses calculées sous forme de tableau.
     *
     * @param gearRatios tableau de proportions
     * @return double[] { intakeSpeed, feederSpeed }
     */
    public static double[] getSpeeds(double[] gearRatios) {
        return new double[]{
            getIntakeSpeed(gearRatios),
            getFeederSpeed(gearRatios)
        };
    }
}
