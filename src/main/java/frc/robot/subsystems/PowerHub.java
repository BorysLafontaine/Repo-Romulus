// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerHub extends SubsystemBase {
  private final PowerDistribution mPowerHub;
  /** Creates a new PowerHub. */
  public PowerHub() {
    mPowerHub = new PowerDistribution(1, ModuleType.kRev);
  }

  public double getVoltage(){
    return mPowerHub.getVoltage();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("battery voltage", getVoltage());
    // This method will be called once per scheduler run
  }
}
