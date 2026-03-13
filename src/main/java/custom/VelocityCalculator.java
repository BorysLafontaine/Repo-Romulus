package custom;
/**
 * VelocityCalculator — Calcule la vélocité de tir en RPM.
 *
 * Principe :
 *  1. Calcule la distance robot → hub
 *  2. Applique une régression quadratique calibrée (v = a·d² + b·d + c)
 *     pour obtenir le RPM idéal à tension nominale
 *  3. Compense la tension batterie actuelle pour maintenir le RPM cible
 *     même quand la batterie faiblit
 *
 * Sortie : RPM à envoyer au ShooterMotor via setVelocityRPM()
 */
public class VelocityCalculator {

    private static final double HUB_X = 182.11; // pouces
    private static final double HUB_Y = 158.84; // pouces

    /** Tension nominale à laquelle les RPM de calibration ont été mesurés */
    private static final double NOMINAL_VOLTAGE = 12.5; // volts

    /**
     * RPM maximum physique du moteur de tir.
     * Sert de garde-fou — VelocityCalculator ne retournera jamais au-delà.
     * Ajuste selon ton moteur (ex: Falcon 500 ≈ 6380 RPM à vide).
     */
    private static final double MAX_RPM = 5600.0;

    // ── Table de calibration : { robotX, robotY, RPM mesurés à NOMINAL_VOLTAGE } ──
    // Mesures à prendre sur le vrai robot à différentes distances du hub.
    private static final double[][] DATA = {
        {  50.0,  50.0, 5400.0 },  // proche du hub  → RPM élevé
        { 100.0,  80.0, 4800.0 },
        { 150.0, 120.0, 4100.0 },
        { 200.0, 160.0, 3500.0 },
        { 250.0, 200.0, 3000.0 },  // loin du hub    → RPM plus bas
        // ⚠️  Remplacer par des mesures réelles sur le robot
    };

    // Coefficients de la courbe v = a·d² + b·d + c (calculés une seule fois)
    private final double coeffA;
    private final double coeffB;
    private final double coeffC;

    public VelocityCalculator() {
        double[] coeffs = fitQuadratic(DATA);
        coeffA = coeffs[0];
        coeffB = coeffs[1];
        coeffC = coeffs[2];
        System.out.printf("Régression RPM : v = %.6f·d² + %.6f·d + %.6f%n", coeffA, coeffB, coeffC);
    }

    // ── API publique ──────────────────────────────────────────────────────────

    /**
     * Calcule le RPM cible à envoyer au moteur de tir.
     *
     * @param x              Position X du robot en pouces
     * @param y              Position Y du robot en pouces
     * @param currentVoltage Tension batterie actuelle en volts
     * @return RPM cible (compensé en tension, clampé entre 0 et MAX_RPM)
     */
    public double calculateRPM(double x, double y, double currentVoltage) {

        // 1. Distance jusqu'au hub
        double distance = distanceToHub(x, y);

        // 2. RPM idéal depuis la courbe (calibré à tension nominale)
        double baseRPM = coeffA * distance * distance + coeffB * distance + coeffC;

        // 3. Compensation tension :
        //    Si batterie faible → moteur tourne plus lentement à même commande
        //    → on augmente la commande RPM proportionnellement
        double voltageCompensation = (currentVoltage > 0.0)
                ? NOMINAL_VOLTAGE / currentVoltage
                : 1.0;

        double compensatedRPM = baseRPM * voltageCompensation;

        // 4. Clamp entre 0 et MAX_RPM
        return Math.max(0.0, Math.min(MAX_RPM, compensatedRPM));
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private double distanceToHub(double x, double y) {
        double dx = HUB_X - x;
        double dy = HUB_Y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Régression quadratique par moindres carrés → coefficients [a, b, c] */
    private double[] fitQuadratic(double[][] data) {
        int n = data.length;
        double[][] X = new double[n][3];
        double[]   Y = new double[n];

        for (int i = 0; i < n; i++) {
            double d = distanceToHub(data[i][0], data[i][1]);
            X[i][0] = d * d;
            X[i][1] = d;
            X[i][2] = 1.0;
            Y[i]    = data[i][2]; // RPM mesuré
        }

        double[][] XtX = multiply(transpose(X), X);
        double[]   XtY = multiply(transpose(X), Y);
        return solve3x3(XtX, XtY);
    }

    // ── Algèbre linéaire interne ──────────────────────────────────────────────

    private double[][] transpose(double[][] M) {
        int r = M.length, c = M[0].length;
        double[][] T = new double[c][r];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                T[j][i] = M[i][j];
        return T;
    }

    private double[][] multiply(double[][] A, double[][] B) {
        int r = A.length, k = B.length, c = B[0].length;
        double[][] R = new double[r][c];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                for (int p = 0; p < k; p++)
                    R[i][j] += A[i][p] * B[p][j];
        return R;
    }

    private double[] multiply(double[][] A, double[] b) {
        int r = A.length, c = b.length;
        double[] R = new double[r];
        for (int i = 0; i < r; i++)
            for (int j = 0; j < c; j++)
                R[i] += A[i][j] * b[j];
        return R;
    }

    private double[] solve3x3(double[][] A, double[] b) {
        int n = 3;
        double[][] M = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) M[i][j] = A[i][j];
            M[i][n] = b[i];
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++)
                if (Math.abs(M[row][col]) > Math.abs(M[pivot][col])) pivot = row;
            double[] tmp = M[col]; M[col] = M[pivot]; M[pivot] = tmp;
            for (int row = col + 1; row < n; row++) {
                double factor = M[row][col] / M[col][col];
                for (int j = col; j <= n; j++)
                    M[row][j] -= factor * M[col][j];
            }
        }
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            x[i] = M[i][n];
            for (int j = i + 1; j < n; j++) x[i] -= M[i][j] * x[j];
            x[i] /= M[i][i];
        }
        return x;
    }
}
