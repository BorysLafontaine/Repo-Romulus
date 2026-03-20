// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.SS_Intake;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    
    private final SS_Intake m_intake = new SS_Intake();
    // ⚠️ port 1 pour ne pas conflit avec le xbox sur port 0
    private final Joystick m_stick = new Joystick(1);
    private static final int kDeployButton     = 1;
    private static final int kRetractButton    = 2;
    private static final int kCompressorButton = 3;
    private static final int kSolenoidButton   = 4;

    public Robot() {
        m_robotContainer = new RobotContainer();   
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}
    @Override
    public void disabledPeriodic() {}
    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}
    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        
    }

    @Override
    public void teleopPeriodic() {  
        // Single solenoid — maintenu tant que bouton enfoncé
        m_intake.setSolenoid(m_stick.getRawButton(kSolenoidButton));
        // Double solenoid — deploy ou retract
        if (m_stick.getRawButtonPressed(kDeployButton)) {
            m_intake.deploy();
        } else if (m_stick.getRawButtonPressed(kRetractButton)) {
            m_intake.retract();
        }
        // Toggle compressor
        if (m_stick.getRawButtonPressed(kCompressorButton)) {
            m_intake.toggleCompressor();
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
 
    @Override
    public void testPeriodic() {}
    @Override
    public void testExit() {}
    @Override
    public void simulationPeriodic() {}
}
