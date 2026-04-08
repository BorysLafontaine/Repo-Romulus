package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SS_TurretAim extends SubsystemBase {

    // =========================
    // HARDWARE
    // =========================
    private static final int MOTOR_ID = 40;

    // =========================
    // TURRET PIVOT OFFSET FROM ROBOT CENTER (meters)
    // X = forward (+) / back (−), Y = left (+) / right (−)
    // =========================
    public static final double TURRET_OFFSET_X = -0.1651; // 6.5" behind center
    public static final double TURRET_OFFSET_Y =  0.0;

    // =========================
    // GEAR RATIO — user-tuned value
    // 5:1 gearbox → 12t driving 155t (but measured with 14t pinion)
    // =========================
    public static final double GEAR_RATIO = 5.0 * (155.0 / 13.0);

    // =========================
    // SOFT LIMITS
    // ⚠️ Going past these WILL rip the cable chain.
    // Range: ±180° (encoder 0 = right side of robot)
    // =========================
    public  static final double SOFT_MIN_DEG = -180.0;
    public  static final double SOFT_MAX_DEG =  180.0;
    private static final double SOFT_MIN_ROT = SOFT_MIN_DEG / 360.0;
    private static final double SOFT_MAX_ROT = SOFT_MAX_DEG / 360.0;

    // =========================
    // PID / FEEDFORWARD — tunable via SmartDashboard at runtime
    // =========================
    private static final double DEFAULT_kP         = 100.0;
    private static final double DEFAULT_kI         =   0.0;
    private static final double DEFAULT_kD         =   5.0;
    private static final double DEFAULT_kS         =   0.25;
    private static final double DEFAULT_kV         =   0.10;
    private static final double DEFAULT_CRUISE_RPS =   120.0;
    private static final double DEFAULT_ACCEL_RPS2 =   1440.0;
    private static final double DEFAULT_JERK_RPS3  =  3600.0;

    // =========================
    // DEADZONE
    // If the error between target and actual is below this, skip commanding the motor.
    // Prevents constant micro-corrections that cause jitter.
    // Tunable at runtime via SmartDashboard.
    // =========================
    private static final double DEFAULT_DEADZONE_DEG = 1.5;

    // =========================
    // ON-TARGET TOLERANCE (for isOnTarget() / shooting gate)
    // =========================
    public static final double ON_TARGET_DEG = 3;

    // =========================
    // MOTOR + REQUESTS
    // =========================
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private final MotionMagicVoltage positionRequest =
        new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);

    private final PositionVoltage trackingRequest =
        new PositionVoltage(0).withEnableFOC(false).withSlot(0);

    // Cached config for runtime PID updates
    private final TalonFXConfiguration cfg = new TalonFXConfiguration();

    // Last commanded target — used for deadzone check and end() hold
    private double lastTargetDeg = 0.0;

    // True while an aim command is actively running
    private boolean aimActive = false;

    // =========================
    // CONSTRUCTOR
    // =========================
    public SS_TurretAim() {

        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        cfg.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_MAX_ROT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_MIN_ROT;

        cfg.Slot0.kP = DEFAULT_kP;
        cfg.Slot0.kI = DEFAULT_kI;
        cfg.Slot0.kD = DEFAULT_kD;
        cfg.Slot0.kS = DEFAULT_kS;
        cfg.Slot0.kV = DEFAULT_kV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = DEFAULT_CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = DEFAULT_ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = DEFAULT_JERK_RPS3;

        motor.getConfigurator().apply(cfg);

        // ⚠️ Assumes turret starts at 0° (encoder 0 = right side of robot).
        motor.setPosition(0.0);

        // Publish tunable defaults to SmartDashboard once
        SmartDashboard.putNumber("Turret/Tune/kP",        DEFAULT_kP);
        SmartDashboard.putNumber("Turret/Tune/kI",        DEFAULT_kI);
        SmartDashboard.putNumber("Turret/Tune/kD",        DEFAULT_kD);
        SmartDashboard.putNumber("Turret/Tune/kS",        DEFAULT_kS);
        SmartDashboard.putNumber("Turret/Tune/kV",        DEFAULT_kV);
        SmartDashboard.putNumber("Turret/Tune/CruiseRPS", DEFAULT_CRUISE_RPS);
        SmartDashboard.putNumber("Turret/Tune/AccelRPS2", DEFAULT_ACCEL_RPS2);
        SmartDashboard.putNumber("Turret/Tune/JerkRPS3",  DEFAULT_JERK_RPS3);
        SmartDashboard.putNumber("Turret/Tune/DeadzoneDeg", DEFAULT_DEADZONE_DEG);
    }

    // =========================
    // SET TARGET ANGLE — static target (MotionMagic profiled move)
    // Skips commanding if within deadzone.
    // =========================
    public void setAngleDegrees(double degrees) {
        double clamped  = MathUtil.clamp(degrees, SOFT_MIN_DEG, SOFT_MAX_DEG);
        double errorDeg = Math.abs(clamped - getAngleDegrees());
        double deadzone = SmartDashboard.getNumber("Turret/Tune/DeadzoneDeg", DEFAULT_DEADZONE_DEG);

        lastTargetDeg = clamped;
        aimActive = true;

        if (errorDeg < deadzone) return; // within deadzone — hold current position

        motor.setControl(positionRequest.withPosition(clamped / 360.0));
    }

    // =========================
    // SET TARGET ANGLE — tracking a moving target (PositionVoltage + velocity FF)
    // Skips commanding if within deadzone.
    // velocityDegPerSec: expected rate of change of the target angle (deg/s).
    // =========================
    public void setAngleDegreesTracking(double degrees, double velocityDegPerSec) {
        double clamped      = MathUtil.clamp(degrees, SOFT_MIN_DEG, SOFT_MAX_DEG);
        double errorDeg     = Math.abs(clamped - getAngleDegrees());
        double deadzone     = SmartDashboard.getNumber("Turret/Tune/DeadzoneDeg", DEFAULT_DEADZONE_DEG);
        double velRotPerSec = velocityDegPerSec / 360.0;

        lastTargetDeg = clamped;

        if (errorDeg < deadzone) return; // within deadzone — hold current position

        motor.setControl(
            trackingRequest
                .withPosition(clamped / 360.0)
                .withVelocity(velRotPerSec)
        );
    }

    public void stop() {
        aimActive = false;
        motor.stopMotor();
    }

    /** Zeroes the encoder at the current physical position. Does not move the motor. */
    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    // =========================
    // GETTERS
    // =========================
    public double getAngleDegrees() {
        return motor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getVelocityDegPerSec() {
        return motor.getVelocity().getValueAsDouble() * 360.0;
    }

    public double getLastTargetDeg() {
        return lastTargetDeg;
    }

    public boolean isOnTarget(double targetDeg) {
        return Math.abs(getAngleDegrees() - targetDeg) < ON_TARGET_DEG;
    }

    // =========================
    // PERIODIC — live PID tuning + diagnostics
    // =========================
    @Override
    public void periodic() {

        // Read tunable values from SmartDashboard and push to controller if changed
        double sdkP        = SmartDashboard.getNumber("Turret/Tune/kP",        DEFAULT_kP);
        double sdkI        = SmartDashboard.getNumber("Turret/Tune/kI",        DEFAULT_kI);
        double sdkD        = SmartDashboard.getNumber("Turret/Tune/kD",        DEFAULT_kD);
        double sdkS        = SmartDashboard.getNumber("Turret/Tune/kS",        DEFAULT_kS);
        double sdkV        = SmartDashboard.getNumber("Turret/Tune/kV",        DEFAULT_kV);
        double sdCruise    = SmartDashboard.getNumber("Turret/Tune/CruiseRPS", DEFAULT_CRUISE_RPS);
        double sdAccel     = SmartDashboard.getNumber("Turret/Tune/AccelRPS2", DEFAULT_ACCEL_RPS2);
        double sdJerk      = SmartDashboard.getNumber("Turret/Tune/JerkRPS3",  DEFAULT_JERK_RPS3);

        boolean pidChanged = sdkP != cfg.Slot0.kP || sdkI != cfg.Slot0.kI
                          || sdkD != cfg.Slot0.kD || sdkS != cfg.Slot0.kS
                          || sdkV != cfg.Slot0.kV;
        boolean profileChanged = sdCruise != cfg.MotionMagic.MotionMagicCruiseVelocity
                              || sdAccel  != cfg.MotionMagic.MotionMagicAcceleration
                              || sdJerk   != cfg.MotionMagic.MotionMagicJerk;

        if (pidChanged || profileChanged) {
            cfg.Slot0.kP = sdkP;
            cfg.Slot0.kI = sdkI;
            cfg.Slot0.kD = sdkD;
            cfg.Slot0.kS = sdkS;
            cfg.Slot0.kV = sdkV;
            cfg.MotionMagic.MotionMagicCruiseVelocity = sdCruise;
            cfg.MotionMagic.MotionMagicAcceleration   = sdAccel;
            cfg.MotionMagic.MotionMagicJerk           = sdJerk;
            motor.getConfigurator().apply(cfg);
        }

        double actualDeg  = getAngleDegrees();
        double velDegS    = getVelocityDegPerSec();
        double errorDeg   = lastTargetDeg - actualDeg;
        double statorA    = motor.getStatorCurrent().getValueAsDouble();
        double supplyV    = motor.getSupplyVoltage().getValueAsDouble();
        boolean onTarget  = isOnTarget(lastTargetDeg);

        // AdvantageKit
        Logger.recordOutput("Turret/AngleDeg",      actualDeg);
        Logger.recordOutput("Turret/TargetDeg",     lastTargetDeg);
        Logger.recordOutput("Turret/ErrorDeg",      errorDeg);
        Logger.recordOutput("Turret/VelocityDegS",  velDegS);
        Logger.recordOutput("Turret/StatorCurrent", statorA);
        Logger.recordOutput("Turret/SupplyVoltage", supplyV);
        Logger.recordOutput("Turret/OnTarget",      onTarget);

        // Elastic / SmartDashboard
        SmartDashboard.putNumber ("Turret/AngleDeg",      actualDeg);
        SmartDashboard.putNumber ("Turret/TargetDeg",     lastTargetDeg);
        SmartDashboard.putNumber ("Turret/ErrorDeg",      errorDeg);
        SmartDashboard.putNumber ("Turret/VelocityDegS",  velDegS);
        SmartDashboard.putNumber ("Turret/StatorCurrent", statorA);
        SmartDashboard.putNumber ("Turret/SupplyVoltage", supplyV);
        SmartDashboard.putBoolean("Turret/OnTarget",      onTarget);
        SmartDashboard.putBoolean("Turret/Active",        aimActive);
    }
}
