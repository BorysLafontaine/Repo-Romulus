// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Constants;
import frc.robot.generated.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Transfer extends SubsystemBase {

  private final TalonFX m_TransferMotor;

  /** Creates a new SS_Shooter. */
  public SS_Transfer() {
    m_TransferMotor = new TalonFX(30);
    
  }

  public void SpinTransfer(){
    m_TransferMotor.set(Constants.RobotConstants.TransferSpeed);
  }

  public void reverseSpinTransfer(){
    m_TransferMotor.set(-Constants.RobotConstants.TransferSpeed);
  }

  public void stopTransfer(){
     m_TransferMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}