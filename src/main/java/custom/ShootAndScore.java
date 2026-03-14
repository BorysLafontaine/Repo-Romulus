package custom;

// import krakenx60; // to fix
import custom.ShooterTracker;
import custom.VelocityCalculator;
/**
 * ShootAndScore — Orchestre le mode tir complet.
 *
 * ─── Flux RPM ─────────────────────────────────────────────────────────────────
 *
 *   VelocityCalculator.calculateRPM(x, y, voltage)
 *           │
 *           │  RPM cible (compensé en tension)
 *           ▼
 *   ShooterMotor.setVelocityRPM(rpm)
 *           │
 *           │  Le moteur (TalonFX, SparkMax…) gère son PID interne pour
 *           │  atteindre et maintenir ce RPM
 *           ▼
 *   ShooterMotor.getCurrentRPM()  ← lu pour valider SPOOLING_UP → FIRING
 *
 * ─── Séquence du mode tir ────────────────────────────────────────────────────
 *
 *   IDLE ──[toggle Button]──▶ SPOOLING_UP ──[RPM atteint]──▶ FIRING
 *    ▲                                                           │
 *    └───────────────────────[toggle Button]─────────────────────┘
 */
public class ShootAndScore {

    private TalonFX m_TalonFX;
    public enum ShootState { IDLE, SPOOLING_UP, FIRING }

    /**
     * Tolérance RPM pour valider que le shooter est à vitesse.
     * Ex: ±150 RPM → le moteur est considéré "à vitesse" si dans cette fenêtre.
     */
    private static final double RPM_TOLERANCE = 150.0;

    // ── Sous-systèmes intégrés ────────────────────────────────────────────────
    private final ShooterTracker     shooterTracker;
    private final VelocityCalculator velocityCalculator;

    // ── Dépendances externes ──────────────────────────────────────────────────
    private final Rollers        rollers;
    private final deploy_intake  deployIntake;
    private final ShooterMotor   shooterMotor;

    // ── État interne ──────────────────────────────────────────────────────────
    private ShootState currentState       = ShootState.IDLE;
    private boolean    ShootToggleBtn = false;

    // ── Constructeur ──────────────────────────────────────────────────────────
    public ShootAndScore(Rollers rollers, deploy_intake deployIntake, ShooterMotor shooterMotor) {
        this.shooterTracker     = new ShooterTracker();
        this.velocityCalculator = new VelocityCalculator();
        this.rollers            = rollers;
        this.deployIntake       = deployIntake;
        this.shooterMotor       = shooterMotor;
    }

    // ── Update — appeler dans periodic() À CHAQUE CYCLE ──────────────────────
    /**
     * @param shootToggleBtn  B5 — bouton toggle du mode tir
     * @param robotX          position X du robot en pouces
     * @param robotY          position Y du robot en pouces
     * @param robotAngle      cap du robot en radians
     * @param batteryVoltage  tension batterie (RobotController.getBatteryVoltage())
     */
    public void update(
            boolean shootToggleBtn,
            double  robotX,
            double  robotY,
            double  robotAngle,
            double  batteryVoltage
    ) {
        // ── 1. Calculs continus — TOUJOURS, peu importe l'état ────────────────
        //    La tourelle vise et le RPM cible est prêt avant même que le driver appuie
        shooterTracker.update(robotX, robotY, robotAngle);
        targetRPM = velocityCalculator.calculateRPM(robotX, robotY, batteryVoltage);

        // ── 2. Gestion du toggle ──────────────────────────────────────────────
        boolean risingEdge = shootToggleBtn && !ShootToggleBtn;
        ShootToggleBtn = shootToggleBtn;

        if (risingEdge) {
            if (currentState == ShootState.IDLE) {
                currentState = ShootState.SPOOLING_UP;
            } else {
                enterIdle();
                return;
            }
        }

        // ── 3. Machine à états ────────────────────────────────────────────────
        switch (currentState) {
            case IDLE        -> handleIdle();
            case SPOOLING_UP -> handleSpoolingUp();
            case FIRING      -> handleFiring();
        }
    }

    // ── Handlers ─────────────────────────────────────────────────────────────

    /** IDLE : shooter arrêté, rollers libres */
    private void handleIdle() {
        shooterMotor.setVelocityRPM(0.0);
        rollers.setFeedingToggle(false);
        rollers.setIntakeFeedToggle(false);
        rollers.setIntakingToggle(false);
    }

    /**
     * SPOOLING_UP : envoie targetRPM au moteur.
     * Rollers OFF — on n'alimente pas avant d'être à vitesse.
     * Transition → FIRING dès que le RPM est dans la tolérance.
     */
    private void handleSpoolingUp() {
        // Envoi du RPM calculé par VelocityCalculator au moteur de tir
        shooterMotor.setVelocityRPM(targetRPM);

        rollers.setFeedingToggle(false);
        rollers.setIntakeFeedToggle(false);
        rollers.setIntakingToggle(false);

        // Transition dès que le RPM réel est dans la tolérance du RPM cible
        if (isAtTargetRPM()) {
            currentState = ShootState.FIRING;
            System.out.printf("✅ RPM atteint (%.0f / %.0f) — FIRING%n",
                              shooterMotor.getCurrentRPM(), targetRPM);
        }
    }

    /**
     * FIRING : feeder + intake (si déployé) alimentent.
     * targetRPM est recalculé en continu → compensation tension en temps réel.
     */
    private void handleFiring() {
        // RPM maintenu en continu (déjà recalculé en haut de update())
        shooterMotor.setVelocityRPM(targetRPM);

        if (deployIntake.isDeployed()) {
            // Intake déployé → intake + feeder simultanés
            rollers.setIntakeFeedToggle(true);
            rollers.setFeedingToggle(false);
            rollers.setIntakingToggle(false);
        } else {
            // Intake non déployé → feeder seul (Rollers bloque aussi automatiquement)
            rollers.setIntakeFeedToggle(false);
            rollers.setFeedingToggle(true);
            rollers.setIntakingToggle(false);
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private void enterIdle() {
        currentState = ShootState.IDLE;
        shooterMotor.setVelocityRPM(0.0);
        rollers.setFeedingToggle(false);
        rollers.setIntakeFeedToggle(false);
        rollers.setIntakingToggle(false);
    }

    /**
     * Vérifie si le RPM actuel est dans la fenêtre de tolérance autour de targetRPM.
     */
    private boolean isAtTargetRPM() {
        return Math.abs(shooterMotor.getCurrentRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    // ── Getters ───────────────────────────────────────────────────────────────
    public ShootState getCurrentState()  { return currentState; }
    public boolean    isIdle()           { return currentState == ShootState.IDLE; }
    public boolean    isSpoolingUp()     { return currentState == ShootState.SPOOLING_UP; }
    public boolean    isFiring()         { return currentState == ShootState.FIRING; }
    public double     getTargetRPM()     { return targetRPM; }
    public double     getTurretAngle()   { return shooterTracker.getCurrentRotation(); }

    // ── Interface ShooterMotor ────────────────────────────────────────────────
    /**
     * Remplace par ton vrai moteur (TalonFX, SparkMax, etc.)
     *
     * Exemple TalonFX (CTRE) :
     *   setVelocityRPM(rpm) → motor.set(TalonFXControlMode.Velocity, rpm / 600.0 * 2048);
     *   getCurrentRPM()     → motor.getSelectedSensorVelocity() * 600.0 / 2048;
     *
     * Exemple SparkMax (REV) :
     *   setVelocityRPM(rpm) → pidController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
     *   getCurrentRPM()     → encoder.getVelocity();
     */
    public interface ShooterMotor {
        /** Envoie le RPM cible au contrôleur PID interne du moteur */
        void   setVelocityRPM(double rpm);
        /** Lit le RPM actuel depuis l'encodeur */
        double getCurrentRPM();
    }
}
