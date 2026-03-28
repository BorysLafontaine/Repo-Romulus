// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_ShotCalculatorWrapper;
import frc.robot.subsystems.SS_Transfer;
import frc.robot.subsystems.SS_TurretFixed;
import frc.robot.commands.IntakeSpin_CMD;
import frc.robot.commands.RollerSpin_CMD;
import frc.robot.commands.ShooterSpin_CMD;
import frc.robot.commands.Transfer_CMD;
import frc.robot.commands.TurretGoToTarget_CMD;
import frc.robot.commands.UltimateEverything_CMD;
import frc.robot.subsystems.SS_Intake;
import frc.robot.subsystems.SS_IntakeMotors;
import frc.robot.subsystems.SS_Rollers;
import frc.robot.commands.toggle_intake_CMD;


public class RobotContainer {

    private SlewRateLimiter xLimiter = new SlewRateLimiter(0.8);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(0.8);
    private SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.5);
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SS_Shooter mShooter = new SS_Shooter();
    private final ShooterSpin_CMD mShooterSpin_CMD = new ShooterSpin_CMD(mShooter);

    private final SS_TurretFixed mTurretFixed = new SS_TurretFixed();
    private final TurretGoToTarget_CMD mTurretGoToTarget_CMD = new TurretGoToTarget_CMD(mTurretFixed);

    private final SS_Intake mIntake = new SS_Intake();
    private final toggle_intake_CMD mtoggle_intake_CMD = new toggle_intake_CMD(mIntake);

    private final SS_IntakeMotors mIntakeMotors = new SS_IntakeMotors();
    private final IntakeSpin_CMD mIntakeSpin_CMD = new IntakeSpin_CMD(mIntakeMotors);

    private final SS_Transfer mTransfer = new SS_Transfer();
    private final Transfer_CMD mTransfer_CMD = new Transfer_CMD(mTransfer); 

    private final SS_Rollers mRollers = new SS_Rollers();
    private final RollerSpin_CMD mRollerSpin_CMD = new RollerSpin_CMD(mRollers); 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SS_ShotCalculatorWrapper m_shotCalc = new SS_ShotCalculatorWrapper(drivetrain);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(yLimiter.calculate(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(omegaLimiter.calculate(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        joystick.x().toggleOnTrue(mShooterSpin_CMD);
        joystick.povRight().toggleOnTrue(mTurretGoToTarget_CMD);
        joystick.y().onTrue(mtoggle_intake_CMD);
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().toggleOnTrue(mIntakeSpin_CMD);
        joystick.b().whileTrue(mIntakeSpin_CMD);
        joystick.b().whileTrue(mRollerSpin_CMD);
        joystick.b().whileTrue(mTransfer_CMD);
        
        // Reset the field-centric heading on left bumper press.
        joystick.povLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
