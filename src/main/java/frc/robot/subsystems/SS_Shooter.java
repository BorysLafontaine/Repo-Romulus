// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Constants;
import frc.robot.generated.Constants.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {

  private final TalonFX m_ShooterMotor;

  /** Creates a new Indexer. */
  public SS_Shooter() {
    m_ShooterMotor = new TalonFX(30);
  }

  public void spinShooter(){
    m_ShooterMotor.set(Constants.RobotConstants.ShooterSpeed);
  }

  public void stopShooter(){
     m_ShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}