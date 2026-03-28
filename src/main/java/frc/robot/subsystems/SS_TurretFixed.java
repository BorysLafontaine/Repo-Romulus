// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_TurretFixed extends SubsystemBase {

  private TalonFX m_turretMotor;

  public static double kP = 0.00925;
  public static double kI = 0.00006;
  public static double kD = 0.0001;
  public static double kF = 0.0;
  public static double nominalVoltage = 12.0;

  private double error = 0.0;
  private double integral = 0.0;
  private double derivative = 0.0;
  private double lastError = 0.0;
  private final double MAX_INTEGRAL = 5000.0;

  private double targetAngle = 0.0;
  private double correctedTargetAngle = 0.0;

  private double output = 0.0;

  // ✅ CORRECT gear math
  // 5:1 gearbox and 12:155 reduction → motor spins ~64.58 times per turret rotation
  private final double gearRatio = (155.0 / 12.0) * 5.0;
  private final double degPerMotorRot = 360.0 / gearRatio;

  public SS_TurretFixed() {
    m_turretMotor = new TalonFX(40);
    m_turretMotor.setSafetyEnabled(false);

    m_turretMotor.setNeutralMode(NeutralModeValue.Coast);

    m_turretMotor.setPosition(0);

    SmartDashboard.putNumber("kP_T", kP);
    SmartDashboard.putNumber("kI_T", kI);
    SmartDashboard.putNumber("kD_T", kD);
    SmartDashboard.putNumber("kF_T", kF);

    SmartDashboard.putNumber("Target Angle", targetAngle);
  }

  public void setTargetAngle(double angleInRad) {
    targetAngle = Math.toDegrees(MathUtil.angleModulus(angleInRad));
  }

  public void turretGoToTarget() {
    double motorRot = m_turretMotor.getPosition().getValueAsDouble();

    // ✅ Convert motor rotations → turret degrees
    double currentPos = motorRot * degPerMotorRot;

    double currentVoltage = SmartDashboard.getNumber("battery voltage", 12.0);

    // Clamp target
    if (targetAngle > 90) {
      correctedTargetAngle = 90;
    } else if (targetAngle < -90) {
      correctedTargetAngle = -90;
    } else {
      correctedTargetAngle = targetAngle;
    }

    m_turretMotor.set(calculate(correctedTargetAngle, currentPos, currentVoltage));
  }

  public void stopTurret() {
    resetController();
    m_turretMotor.stopMotor();
  }

  @Override
  public void periodic() {
    kP = SmartDashboard.getNumber("kP_T", 0);
    kI = SmartDashboard.getNumber("kI_T", 0);
    kD = SmartDashboard.getNumber("kD_T", 0);
    kF = SmartDashboard.getNumber("kF_T", 0);

    targetAngle = SmartDashboard.getNumber("Target Angle", 0);

    double motorRot = m_turretMotor.getPosition().getValueAsDouble();
    double currentDeg = motorRot * degPerMotorRot;

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Integral", integral);
    SmartDashboard.putNumber("Derivative", derivative);
    SmartDashboard.putNumber("Current pos in deg", currentDeg);
  }

  public double calculate(double target, double current, double currentVoltage) {
    error = target - current;

    integral += error;
    if (Math.abs(integral) > MAX_INTEGRAL) {
      integral = MAX_INTEGRAL * Math.signum(integral);
    }

    // ✅ FIXED derivative
    derivative = (error - lastError);

    // ✅ Safe sign (avoids divide by zero)
    double sign = (Math.abs(error) > 1e-5) ? Math.signum(error) : 0.0;

    output = kP * error
           + kI * integral
           + kD * derivative
           + kF * target;

    lastError = error;

    // Clamp output
    if (Math.abs(output) > 0.5) {
      output = 0.5 * Math.signum(output);
    }

    return output;
  }

  public void resetController() {
    integral = 0.0;
    derivative = 0.0;
    lastError = 0.0;
  }
}
