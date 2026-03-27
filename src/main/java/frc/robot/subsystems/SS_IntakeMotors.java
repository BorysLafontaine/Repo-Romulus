// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Constants;
import frc.robot.generated.Constants.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_IntakeMotors extends SubsystemBase {

  private final TalonFX m_IRollerMotor;

  /** Creates a new SS_Shooter. */
  public SS_IntakeMotors() {
    m_IRollerMotor = new TalonFX(20);
  }

  public void SpinIRoller(){
    m_IRollerMotor.set(Constants.RobotConstants.IRollerSpeed);
  }

  public void stopIRoller(){
     m_IRollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}