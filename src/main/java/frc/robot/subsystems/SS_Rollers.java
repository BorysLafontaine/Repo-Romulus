// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Constants;
import frc.robot.generated.Constants.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Rollers extends SubsystemBase {

  private final TalonFX m_RollerMotor;

  /** Creates a new SS_Shooter. */
  public SS_Rollers() {
    m_RollerMotor = new TalonFX(21);
  }

  public void spinRoller(){
    m_RollerMotor.set(-Constants.RobotConstants.RollerSpeed);
  }

  public void reverseSpinRoller(){
    m_RollerMotor.set(Constants.RobotConstants.RollerSpeed);
  }

  public void stopRoller(){
     m_RollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}