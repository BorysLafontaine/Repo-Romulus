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
  /** Creates a new SS_TurretFixed. */

  private TalonFX m_turretMotor;

  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kF = 0.0;
  public static double nominalVoltage = 12.0;

  private double error = 0.0;
  private double integral = 0.0;
  private double derivative = 0.0;
  private double lastError = 0.0;
  private double MAX_INTEGRAL = 5000.0;

  private double targetAngle = 0.0;
  private double correctedTargetAngle = 0.0;

  private double output = 0.0;

  private final double gearRatio = (155.0 / 12.0) * 5;
  private final double degPerRot = (360 * gearRatio);

  public SS_TurretFixed() {
    m_turretMotor = new TalonFX(40);
    m_turretMotor.setSafetyEnabled(false);

    m_turretMotor.setNeutralMode(NeutralModeValue.Brake);

    m_turretMotor.setPosition(0);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kF", kF);

    SmartDashboard.putNumber("Target Angle", targetAngle);
  }

  public void setTargetAngle(double angleInRad) {
    targetAngle = Math.toDegrees(MathUtil.angleModulus(angleInRad));
  }

  public void turretGoToTarget() {
    double currentPos = m_turretMotor.getPosition().getValueAsDouble() * degPerRot;
    double currentVoltage = SmartDashboard.getNumber("battery voltage", 0);
    if (targetAngle > 90) {
      correctedTargetAngle = 90;
    } else if (targetAngle < -90) {
      correctedTargetAngle = -90;
    } else correctedTargetAngle = targetAngle;

    m_turretMotor.set(calculate(correctedTargetAngle, currentPos, currentVoltage));
  }

  public void stopTurret() {
    resetController();
    m_turretMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kP = SmartDashboard.getNumber("kP_T", 0);
    kI = SmartDashboard.getNumber("kI_T", 0);
    kD = SmartDashboard.getNumber("kD_T", 0);
    kF = SmartDashboard.getNumber("kF_T", 0);

    targetAngle = SmartDashboard.getNumber("Target Angle", 0);

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Integral", integral);
    SmartDashboard.putNumber("Derivative", derivative);
    SmartDashboard.putNumber("Current pos in deg",  m_turretMotor.getPosition().getValueAsDouble() / degPerRot);
  }

  public double calculate(double target, double current, double currentVoltage) {
    error = target - current;
    integral += error;
    if (Math.abs(integral) > MAX_INTEGRAL) {
        integral = MAX_INTEGRAL * Math.signum(integral);
    }
    derivative = (error + lastError);
    output = kP * error + kI * integral + kD * derivative + kF * target + (error / Math.abs(error)) * (currentVoltage / nominalVoltage);
    lastError = error;

    if (Math.abs(output) > 1) {
        output = 1 * Math.signum(output);
    }
    return output;
    }

    public void resetController() {
        integral = 0.0;
        derivative = 0.0;
    }
}
