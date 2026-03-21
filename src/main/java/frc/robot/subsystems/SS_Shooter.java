// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {

  private final TalonFX m_LeadShooterMotor;
  private final TalonFX m_FollowShooterMotor;


//PIDFS Variables
  public static double kP = 0.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kF = 0.0;
  public static double kS = 0.0;
  public static double nominalVoltage = 12.0;

  private double error = 0.0;
  private double integral = 0.0;
  private double derivative = 0.0;
  private double lastError = 0.0;
  private double MAX_INTEGRAL = 5000.0;
  private double dt = 0.0;

  private double output = 0.0;

  //Speed tuning constant
  private double RPM = 0.0;

  /** Creates a new SS_Shooter. */
  public SS_Shooter() {
    m_LeadShooterMotor = new TalonFX(31);
    m_FollowShooterMotor = new TalonFX(32);
    m_FollowShooterMotor.setControl(new Follower(m_LeadShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    m_LeadShooterMotor.setSafetyEnabled(true);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kF", kF);
    SmartDashboard.putNumber("kS", kS);
    SmartDashboard.putNumber("Target RPM", RPM);
  }

  public void spinShooter(){
    double currentVel = 60 * m_LeadShooterMotor.getVelocity().getValueAsDouble();
    double currentVoltage = SmartDashboard.getNumber("battery voltage", 0);
    m_LeadShooterMotor.set(calculate(RPM, currentVel, currentVoltage));
  }

  public void stopShooter(){
    resetController();
    m_LeadShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kP = SmartDashboard.getNumber("kP", 0);
    kI = SmartDashboard.getNumber("kI", 0);
    kD = SmartDashboard.getNumber("kD", 0);
    kF = SmartDashboard.getNumber("kF", 0);
    kS = SmartDashboard.getNumber("kS", 0);
    RPM = SmartDashboard.getNumber("Target RPM", 0);

    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Integral", integral);
    SmartDashboard.putNumber("Derivative", derivative);
    SmartDashboard.putNumber("Current vel", 60 * m_LeadShooterMotor.getVelocity().getValueAsDouble());
  }

  public double calculate(double target, double current, double currentVoltage) {
      error = target - current;
      dt = Timer.getFPGATimestamp() / (10^-9);
      integral += error;
      if (Math.abs(integral) > MAX_INTEGRAL) {
          integral = MAX_INTEGRAL * Math.signum(integral);
      }
      derivative = (error - lastError) / dt;
      output = kP * error + kI * integral + kD * derivative + kF * target + kS * Math.signum(error) + ((error / Math.abs(error)) * (currentVoltage / nominalVoltage));
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