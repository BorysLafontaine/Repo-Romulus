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
    // ⚠️ Set your CAN ID and bus name
    // =========================
    private static final int    MOTOR_ID = 40;
    // =========================
    // TURRET PIVOT OFFSET FROM ROBOT CENTER
    // ⚠️ Measure these on the physical robot (meters).
    // X = forward (+) / back (−) from center
    // Y = left  (+) / right(−) from center
    // Used in TurretAimAtHub_CMD to compute angle from the actual pivot,
    // not the robot center — matters most when the turret is far off-center.
    // =========================
    public static final double TURRET_OFFSET_X = -0.1651; // ⚠️ fill in
    public static final double TURRET_OFFSET_Y = 0.0; // ⚠️ fill in

    // =========================
    // GEAR RATIO
    // 4:1 gearbox → 12t driving 155t pulley
    // Total: 5 × (155 / 12) = 51.667:1
    // =========================
    public static final double GEAR_RATIO = 5.0 * (155.0 / 12.0); // 51.6667

    // =========================
    // SOFT LIMITS
    // ⚠️ Going past these WILL rip the cable chain
    // Stored as mechanism rotations (degrees / 360)
    // =========================
    public  static final double SOFT_MIN_DEG = -95.0;
    public  static final double SOFT_MAX_DEG = 275.0;
    private static final double SOFT_MIN_ROT = SOFT_MIN_DEG / 360.0; // -0.75
    private static final double SOFT_MAX_ROT = SOFT_MAX_DEG / 360.0; //  0.25

    // =========================
    // MOTION MAGIC CONFIG
    // Tune kP first (increase until oscillation, then back off).
    // CRUISE/ACCEL in mechanism rotations/s and rotations/s²
    // =========================
    private static final double kP         = 80.0;
    private static final double kI         =  0.0;
    private static final double kD         =  2.0;
    private static final double kS         =  0.25; // static friction feedforward (V)
    private static final double CRUISE_RPS =  3.0;  // rot/s  — tune for your robot
    private static final double ACCEL_RPS2 = 12.0;  // rot/s²
    private static final double JERK_RPS3  = 60.0;  // rot/s³ — smooths start/stop

    // On-target tolerance
    public static final double ON_TARGET_DEG = 1.0;

    // =========================
    // MOTOR + REQUEST
    // =========================
    private final TalonFX motor = new TalonFX(MOTOR_ID);

    // MotionMagicVoltage gives smooth profiled motion to a position
    private final MotionMagicVoltage positionRequest =
        new MotionMagicVoltage(0).withEnableFOC(false);

    // =========================
    // CONSTRUCTOR
    // =========================
    public SS_TurretAim() {

        var cfg = new TalonFXConfiguration();

        // Motor is physically reversed
        cfg.MotorOutput.Inverted    = InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Tell Phoenix the sensor-to-mechanism ratio so position/velocity
        // signals are already in mechanism (turret) rotations
        cfg.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // Hardware soft limits — last line of defense before cable chain rips
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_MAX_ROT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_MIN_ROT;

        // PID slot 0
        cfg.Slot0.kP = kP;
        cfg.Slot0.kI = kI;
        cfg.Slot0.kD = kD;
        cfg.Slot0.kS = kS;

        // Motion Magic profile
        cfg.MotionMagic.MotionMagicCruiseVelocity = CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = ACCEL_RPS2;
        cfg.MotionMagic.MotionMagicJerk           = JERK_RPS3;

        motor.getConfigurator().apply(cfg);

        // ⚠️ Assumes turret starts at 0° (facing forward) on boot.
        // If your robot homes to a limit switch, zero it there instead.
        motor.setPosition(0.0);
    }

    // =========================
    // SET TARGET ANGLE
    // Input: degrees, will be clamped to [SOFT_MIN_DEG, SOFT_MAX_DEG]
    // =========================
    public void setAngleDegrees(double degrees) {
        double clamped = MathUtil.clamp(degrees, SOFT_MIN_DEG, SOFT_MAX_DEG);
        motor.setControl(positionRequest.withPosition(clamped / 360.0));
        Logger.recordOutput("Turret/TargetDeg", clamped);
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

    public boolean isOnTarget(double targetDeg) {
        return Math.abs(getAngleDegrees() - targetDeg) < ON_TARGET_DEG;
    }

    // =========================
    // PERIODIC
    // =========================
    @Override
    public void periodic() {
        Logger.recordOutput("Turret/AngleDeg",     getAngleDegrees());
        Logger.recordOutput("Turret/VelocityDegS",
            motor.getVelocity().getValueAsDouble() * 360.0);
        Logger.recordOutput("Turret/StatorCurrent",
            motor.getStatorCurrent().getValueAsDouble());
    }
}