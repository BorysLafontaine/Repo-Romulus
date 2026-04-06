package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SS_Shooter extends SubsystemBase {

    private static final int LEAD_MOTOR_ID   = 31;
    private static final int FOLLOW_MOTOR_ID = 32;

    // =========================
    // PID + FEEDFORWARD — ALREADY TUNED, do not change without re-tuning
    // kF: RPM-proportional feedforward (kF * targetRPM added to output)
    // kS: static friction offset (sign-matched)
    // Voltage FF: sign(error) * (Vbatt / 12V) — baked into calculate()
    // =========================
    private static final double kP              = 0.0003;
    private static final double kI              = 0.00005;
    private static final double kD              = 0.0000001;
    private static final double kF              = 0.000171;
    private static final double kS              = 0.0145;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MAX_INTEGRAL    = 5000.0;

    // Robot loop period for derivative. Original code had `dt = Timer.getFPGATimestamp() / (10^-9)`
    // which is wrong: Java ^ is XOR, not power. 10^-9 in Java = -3 (bitwise), making dt ≈ -33.
    // The derivative term contribution with kD=0.0000001 is negligible either way, but fixed here.
    private static final double DT = 0.02; // 50 Hz robot loop

    // =========================
    // ON-SPEED TOLERANCE
    // "Ready to shoot" window in RPM. Tighten for more accuracy, widen for faster shooting.
    // =========================
    public static final double ON_SPEED_RPM = 75.0;

    // =========================
    // HARDWARE
    // =========================
    private final TalonFX leadMotor;
    private final TalonFX followMotor;

    // =========================
    // CONTROLLER STATE
    // =========================
    private double integral  = 0.0;
    private double lastError = 0.0;
    private double targetRPM = 0.0;

    // =========================
    // CONSTRUCTOR
    // =========================
    public SS_Shooter() {
        leadMotor   = new TalonFX(LEAD_MOTOR_ID);
        followMotor = new TalonFX(FOLLOW_MOTOR_ID);

        followMotor.setControl(new Follower(leadMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        leadMotor.setSafetyEnabled(false);
        leadMotor.setNeutralMode(NeutralModeValue.Coast);
        followMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // =========================
    // PRIMARY API
    // Call once per cycle from execute() with the desired RPM.
    // Reads battery voltage from the motor's own supply signal — no SmartDashboard dependency.
    // =========================
    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        double currentRPM     = getCurrentRPM();
        double currentVoltage = leadMotor.getSupplyVoltage().getValueAsDouble();
        leadMotor.set(calculate(rpm, currentRPM, currentVoltage));
    }

    public void stopShooter() {
        resetController();
        targetRPM = 0.0;
        leadMotor.stopMotor();
    }

    // =========================
    // FIXED-SPEED PRESETS (kept for manual override commands)
    // =========================
    public void spinShooter()      { setTargetRPM(2375); }
    public void closeSpinShooter() { setTargetRPM(1500); }
    public void farSpinShooter()   { setTargetRPM(3075); }

    // =========================
    // GETTERS
    // =========================
    public double getCurrentRPM() {
        // TalonFX reports rps; multiply by 60 to get RPM
        return leadMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /** True when spinning within ON_SPEED_RPM of the last commanded target. */
    public boolean isAtSpeed() {
        return isAtSpeed(targetRPM);
    }

    /** True when spinning within ON_SPEED_RPM of the given target. */
    public boolean isAtSpeed(double target) {
        if (target < 10.0) return false; // not meaningfully commanded
        return Math.abs(getCurrentRPM() - target) < ON_SPEED_RPM;
    }

    // =========================
    // PID CALCULATE
    // Preserved original tuned behavior with two bug fixes:
    //   1. dt corrected (was Java XOR, not exponent)
    //   2. Battery voltage read from motor signal, not SmartDashboard
    //   3. sign(0) guard on the voltage feedforward term
    // =========================
    public double calculate(double target, double current, double currentVoltage) {
        double error     = target - current;
        integral        += error; // no DT — matches original tuning (integral per-tick, not per-second)
        integral         = MathUtil.clamp(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        double derivative = (error - lastError) / DT;

        double sign      = Math.signum(error);
        double voltageFF = sign * (currentVoltage / NOMINAL_VOLTAGE);

        double output = kP * error
                      + kI * integral
                      + kD * derivative
                      + kF * target
                      + kS * sign
                      + voltageFF;

        lastError = error;
        return MathUtil.clamp(output, -1.0, 1.0);
    }

    public void resetController() {
        integral  = 0.0;
        lastError = 0.0;
    }

    // =========================
    // PERIODIC
    // =========================
    @Override
    public void periodic() {
        double currentRPM = getCurrentRPM();
        double errorRPM   = targetRPM - currentRPM;

        Logger.recordOutput("Shooter/TargetRPM",  targetRPM);
        Logger.recordOutput("Shooter/CurrentRPM", currentRPM);
        Logger.recordOutput("Shooter/ErrorRPM",   errorRPM);
        Logger.recordOutput("Shooter/AtSpeed",    isAtSpeed());
        Logger.recordOutput("Shooter/Integral",   integral);

        SmartDashboard.putNumber ("Shooter/TargetRPM",  targetRPM);
        SmartDashboard.putNumber ("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber ("Shooter/ErrorRPM",   errorRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed",    isAtSpeed());
    }
}
