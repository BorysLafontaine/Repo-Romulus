package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SS_Shooter extends SubsystemBase {

    private static final int LEAD_MOTOR_ID   = 31;
    private static final int FOLLOW_MOTOR_ID = 32;

    // =========================
    // PID / FEEDFORWARD DEFAULTS — tunable via SmartDashboard at runtime
    // TalonFX VelocityVoltage slot 0:
    //   kP  — output per RPS of error                  (V / RPS)
    //   kI  — output per accumulated RPS error          (V / (RPS * s))
    //   kD  — output per RPS/s of error derivative      (V / (RPS/s))
    //   kV  — feedforward per RPS of target             (V / RPS) — start here first
    //   kS  — static friction offset                    (V)
    // Start tuning: set kP=0, kI=0, kD=0, increase kV until speed is close,
    // then add kP to remove steady-state error.
    // =========================
    private static final double DEFAULT_kP = 0.005;
    private static final double DEFAULT_kI = 0.0;
    private static final double DEFAULT_kD = 0.0;
    private static final double DEFAULT_kV = 0.12;  // V per RPS — 12V / (max ~100 RPS)
    private static final double DEFAULT_kS = 0.1;

    // =========================
    // ON-SPEED TOLERANCE
    // =========================
    public static final double ON_SPEED_RPM = 75.0;

    // =========================
    // DISTANCE → RPM LOOKUP TABLE
    // Key = distance to hub (meters), Value = target RPM
    // ⚠️ Anchors from tuned presets. Calibrate by shooting from known distances.
    // =========================
    private final InterpolatingDoubleTreeMap rpmLUT = new InterpolatingDoubleTreeMap();

    // =========================
    // HARDWARE
    // =========================
    private final TalonFX leadMotor   = new TalonFX(LEAD_MOTOR_ID);
    private final TalonFX followMotor = new TalonFX(FOLLOW_MOTOR_ID);

    // VelocityVoltage: TalonFX closed-loop velocity control in volts
    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withEnableFOC(false).withSlot(0);

    // Cached config for live gain updates
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();

    private double targetRPM = 0.0;

    // =========================
    // CONSTRUCTOR
    // =========================
    public SS_Shooter() {

        // Follower spins opposite to lead (wheels face each other)
        followMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        cfg.MotorOutput.Inverted    = InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Slot 0 — used by VelocityVoltage
        cfg.Slot0.kP = DEFAULT_kP;
        cfg.Slot0.kI = DEFAULT_kI;
        cfg.Slot0.kD = DEFAULT_kD;
        cfg.Slot0.kV = DEFAULT_kV;
        cfg.Slot0.kS = DEFAULT_kS;

        leadMotor.getConfigurator().apply(cfg);
        followMotor.getConfigurator().apply(new TalonFXConfiguration()); // default for follower

        // =========================
        // RPM LOOKUP TABLE
        // Distance (m) → RPM
        // Anchors: 1.5m=1500, 3.0m=2375, 5.0m=3075
        // =========================
        rpmLUT.put(0.5,  1650.0);
        rpmLUT.put(1.0,  1850.0);
        rpmLUT.put(1.5,  1950.0); // ← tuned anchor +250
        rpmLUT.put(2.0,  2350.0);
        rpmLUT.put(2.5,  2625.0);
        rpmLUT.put(3.0,  2825.0); // ← tuned anchor +250
        rpmLUT.put(4.0,  3200.0);
        rpmLUT.put(5.0,  3525.0); // ← tuned anchor +250
        rpmLUT.put(6.5,  3750.0);
        rpmLUT.put(8.0,  4050.0);
        rpmLUT.put(10.0, 4350.0);

        // Publish defaults once so they appear in Elastic/SmartDashboard
        SmartDashboard.putNumber("Shooter/Tune/kP", DEFAULT_kP);
        SmartDashboard.putNumber("Shooter/Tune/kI", DEFAULT_kI);
        SmartDashboard.putNumber("Shooter/Tune/kD", DEFAULT_kD);
        SmartDashboard.putNumber("Shooter/Tune/kV", DEFAULT_kV);
        SmartDashboard.putNumber("Shooter/Tune/kS", DEFAULT_kS);
    }

    // =========================
    // PRIMARY API
    // =========================

    /** Spin to the RPM from the LUT at the given distance (meters). */
    public void setRPMFromDistance(double distanceMeters) {
        setTargetRPM(rpmLUT.get(distanceMeters));
    }

    /** Spin to an explicit RPM target using TalonFX closed-loop velocity. */
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        double targetRPS = rpm / 60.0;
        leadMotor.setControl(velocityRequest.withVelocity(targetRPS));
    }

    public void stopShooter() {
        targetRPM = 0.0;
        leadMotor.stopMotor();
    }

    // =========================
    // GETTERS
    // =========================
    public double getCurrentRPM() {
        return leadMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtSpeed() {
        if (targetRPM < 10.0) return false;
        return Math.abs(getCurrentRPM() - targetRPM) < ON_SPEED_RPM;
    }

    public double getLUTRpm(double distanceMeters) {
        return rpmLUT.get(distanceMeters);
    }

    // No-op — TalonFX manages its own integrator internally
    public void resetController() {}

    // =========================
    // PERIODIC — live gain tuning + diagnostics
    // =========================
    @Override
    public void periodic() {

        // Read tunable gains and push to controller if changed
        double sdkP = SmartDashboard.getNumber("Shooter/Tune/kP", DEFAULT_kP);
        double sdkI = SmartDashboard.getNumber("Shooter/Tune/kI", DEFAULT_kI);
        double sdkD = SmartDashboard.getNumber("Shooter/Tune/kD", DEFAULT_kD);
        double sdkV = SmartDashboard.getNumber("Shooter/Tune/kV", DEFAULT_kV);
        double sdkS = SmartDashboard.getNumber("Shooter/Tune/kS", DEFAULT_kS);

        if (sdkP != cfg.Slot0.kP || sdkI != cfg.Slot0.kI || sdkD != cfg.Slot0.kD
                || sdkV != cfg.Slot0.kV || sdkS != cfg.Slot0.kS) {
            cfg.Slot0.kP = sdkP;
            cfg.Slot0.kI = sdkI;
            cfg.Slot0.kD = sdkD;
            cfg.Slot0.kV = sdkV;
            cfg.Slot0.kS = sdkS;
            leadMotor.getConfigurator().apply(cfg.Slot0);
        }

        double currentRPM = getCurrentRPM();
        double errorRPM   = targetRPM - currentRPM;

        Logger.recordOutput("Shooter/TargetRPM",     targetRPM);
        Logger.recordOutput("Shooter/CurrentRPM",    currentRPM);
        Logger.recordOutput("Shooter/ErrorRPM",      errorRPM);
        Logger.recordOutput("Shooter/AtSpeed",       isAtSpeed());
        Logger.recordOutput("Shooter/StatorCurrent", leadMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber ("Shooter/TargetRPM",  targetRPM);
        SmartDashboard.putNumber ("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber ("Shooter/ErrorRPM",   errorRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed",    isAtSpeed());

        // Graph-friendly: both on the same key namespace so Elastic can overlay them
        SmartDashboard.putNumber("Shooter/Graph/TargetRPM",  targetRPM);
        SmartDashboard.putNumber("Shooter/Graph/CurrentRPM", currentRPM);
    }
}
