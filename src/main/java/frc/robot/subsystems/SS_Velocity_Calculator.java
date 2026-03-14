// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Velocity_Calculator extends SubsystemBase {
  // Config Variables
  private static final double HUB_X = 182.11; // pouces
  private static final double HUB_Y = 158.84; // pouces

  private final TalonFX mShoutingMotor;

  /** Creates a new SS_Velocity_Calculator. */
  public SS_Velocity_Calculator() {
    mShoutingMotor = new TalonFX(xyz); // to fix
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
