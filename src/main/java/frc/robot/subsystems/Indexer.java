// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Constants.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private final TalonFX m_IndexerMotor;

  /** Creates a new Indexer. */
  public Indexer() {
    m_IndexerMotor = new TalonFX(30);
  }

  public void spinIndexer(){
    m_IndexerMotor.set(RobotConstants.IndexSpeed);
  }

  public void stopIndexer(){
     m_IndexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}