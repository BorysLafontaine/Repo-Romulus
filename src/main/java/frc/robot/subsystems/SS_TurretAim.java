package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
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
    // GEAR RATIO
    // 5:1 gearbox → 12t driving 155t pulley
    // Total: 5 × (155 / 12) = 51.667:1
    // =========================
    public static final double GEAR_RATIO = 5.0 * (155.0 / 12.0); // 51.667

    // =========================
    // SOFT LIMITS
    // ⚠️ Going past these WILL rip the cable chain.
    // Range: ±180° (encoder 0 = right side of robot)
    // =========================
    public  static final double SOFT_MIN_DEG = -180.0;
    public  static final double SOFT_MAX_DEG =  180.0;
    private static final double SOFT_MIN_ROT = SOFT_MIN_DEG / 360.0; // -0.5
    private static final double SOFT_MAX_ROT = SOFT_MAX_DEG / 360.0; //  0.5

    // =========================
    // PID / FEEDFORWARD
    // kP: increase until oscillation, then halve.
    // kV: velocity feedforward (V per mechanism rot/s) — start low, increase until
    //     the turret stops lagging behind fast-moving targets.
    // kS: static friction — minimum voltage to move the turret.
    // =========================
    private static final double kP = 80.0;
    private static final double kI =  0.0;
    private static final double kD =  2.0;
    private static final double kS =  0.25;
    private static final double kV =  0.10; // V/(rot/s) at mechanism — tune on robot

    // =========================
    // MOTION MAGIC PROFILE
    // Used for large moves (e.g., initial startup aim).
    // Cruise/accel in mechanism rotations/s and rot/s².
    // =========================
    private static final double CRUISE_RPS =  3.0;
    private static final double ACCEL_RPS2 = 12.0;
    private static final double JERK_RPS3  = 60.0;

    // =========================
    // ON-TARGET TOLERANCE
    // =========================
    public static final double ON_TARGET_DEG = 1.5;

    // =========================
    // MOTOR + REQUESTS
    // Two control modes:
    //   positionRequest  — MotionMagicVoltage: smooth profiled move, good for large steps
    //   trackingRequest  — PositionVoltage w/ velocity FF: best for continuously moving targets
    // =========================
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    private final MotionMagicVoltage positionRequest =
        new MotionMagicVoltage(0).withEnableFOC(false).withSlot(0);

    private final PositionVoltage trackingRequest =
        new PositionVoltage(0).withEnableFOC(false).withSlot(0);

    // =========================
    // CONSTRUCTOR
    // =========================
    public SS_TurretAim() {

        var cfg = new TalonFXConfiguration();

        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        // Brake: turret holds position when no command is running — prevents drift
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Phoenix knows position/velocity are already in mechanism (turret) rotations
        cfg.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // Hardware soft limits — last line of defense before cable chain rips
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_MAX_ROT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_MIN_ROT;

        // PID slot 0 — used by both MotionMagic and PositionVoltage
        cfg.Slot0.kP = kP;
        cfg.Slot0.kI = kI;
        cfg.Slot0.kD = kD;
        cfg.Slot0.kS = kS;
        cfg.Slot0.kV = kV; // velocity feedforward — MotionMagic and PositionVoltage both use it

        // Motion Magic profile (used for large initial moves)
        cfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = JERK_RPS3;

        motor.getConfigurator().apply(cfg);

        // ⚠️ Assumes turret starts at 0° (encoder 0 = right side of robot).
        motor.setPosition(0.0);
    }

    // =========================
    // SET TARGET ANGLE — static target (MotionMagic profiled move)
    // Good for: initial large moves, failsafe commands.
    // =========================
    public void setAngleDegrees(double degrees) {
        double clamped = MathUtil.clamp(degrees, SOFT_MIN_DEG, SOFT_MAX_DEG);
        motor.setControl(positionRequest.withPosition(clamped / 360.0));
    }

    // =========================
    // SET TARGET ANGLE — tracking a moving target (PositionVoltage + velocity FF)
    // Good for: hub tracking while driving, SOTM mode.
    // velocityDegPerSec: expected rate of change of the target angle (deg/s).
    //   Pass 0 to disable velocity feedforward (same as setAngleDegrees but no profile).
    // =========================
    public void setAngleDegreesTracking(double degrees, double velocityDegPerSec) {
        double clamped     = MathUtil.clamp(degrees, SOFT_MIN_DEG, SOFT_MAX_DEG);
        double velRotPerSec = velocityDegPerSec / 360.0;
        motor.setControl(
            trackingRequest
                .withPosition(clamped / 360.0)
                .withVelocity(velRotPerSec)
        );
    }

    public void stop() {
        motor.stopMotor();
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

    public boolean isOnTarget(double targetDeg) {
        return Math.abs(getAngleDegrees() - targetDeg) < ON_TARGET_DEG;
    }

    // =========================
    // PERIODIC
    // =========================
    @Override
    public void periodic() {
        Logger.recordOutput("Turret/AngleDeg",      getAngleDegrees());
        Logger.recordOutput("Turret/VelocityDegS",  getVelocityDegPerSec());
        Logger.recordOutput("Turret/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Turret/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble());
    }
}
