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

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {

    private SlewRateLimiter xLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter omegaLimiter = new SlewRateLimiter(4.0);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SS_Shooter mShooter = new SS_Shooter();
    private final SS_TurretFixed mTurretFixed = new SS_TurretFixed();
    private final SS_Intake mIntake = new SS_Intake();
    private final SS_IntakeMotors mIntakeMotors = new SS_IntakeMotors();
    private final SS_Transfer mTransfer = new SS_Transfer();
    private final SS_Rollers mRollers = new SS_Rollers();

    // Commands
    private final ShooterSpin_CMD mShooterSpin_CMD = new ShooterSpin_CMD(mShooter);
    private final CloseShooterSpin_CMD mCloseShooterSpin_CMD = new CloseShooterSpin_CMD(mShooter);
    private final FarShooterSpin_CMD mFarShooterSpin_CMD = new FarShooterSpin_CMD(mShooter);

    private final TurretGoToTarget_CMD mTurretGoToTarget_CMD = new TurretGoToTarget_CMD(mTurretFixed);

    private final toggle_intake_CMD mtoggle_intake_CMD = new toggle_intake_CMD(mIntake);

    private final IntakeSpin_CMD mIntakeSpin_CMD = new IntakeSpin_CMD(mIntakeMotors);
    private final ReverseIntakeSpin_CMD mReverseIntakeSpin_CMD = new ReverseIntakeSpin_CMD(mIntakeMotors);

    private final Transfer_CMD mTransfer_CMD = new Transfer_CMD(mTransfer);
    private final ReverseTransfer_CMD mReverseTransfer_CMD = new ReverseTransfer_CMD(mTransfer);

    private final RollerSpin_CMD mRollerSpin_CMD = new RollerSpin_CMD(mRollers);
    private final ReverseRollerSpin_CMD mReverseRollerSpin_CMD = new ReverseRollerSpin_CMD(mRollers);

    // Controller
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Default drive
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // 🔥 Heading hold drive
    private final SwerveRequest.FieldCentricFacingAngle driveStraight =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(xLimiter.calculate(-joystick.getLeftY()) * MaxSpeed)
                     .withVelocityY(yLimiter.calculate(-joystick.getLeftX()) * MaxSpeed)
                     .withRotationalRate(omegaLimiter.calculate(-joystick.getRightX()) * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.x().toggleOnTrue(mShooterSpin_CMD);
        joystick.povDown().toggleOnTrue(mCloseShooterSpin_CMD);
        joystick.povUp().toggleOnTrue(mFarShooterSpin_CMD);

        joystick.povRight().toggleOnTrue(mTurretGoToTarget_CMD);

        joystick.y().onTrue(mtoggle_intake_CMD);

        joystick.rightBumper().toggleOnTrue(mIntakeSpin_CMD);

        joystick.leftBumper().whileTrue(mIntakeSpin_CMD);
        joystick.leftBumper().whileTrue(mRollerSpin_CMD);
        joystick.leftBumper().whileTrue(mTransfer_CMD);

        joystick.b().whileTrue(mReverseIntakeSpin_CMD);
        joystick.b().whileTrue(mReverseRollerSpin_CMD);
        joystick.b().whileTrue(mReverseTransfer_CMD);

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.povLeft().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();

        // Mutable container for heading capture
        final Rotation2d[] targetHeading = new Rotation2d[1];

        return Commands.sequence(
            // Reset heading
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

            // Capture heading AFTER reset
            Commands.runOnce(() ->
                targetHeading[0] = drivetrain.getState().Pose.getRotation().plus(Rotation2d.fromRadians(Math.PI))
            ),

            // Drive straight with heading hold
            drivetrain.applyRequest(() ->
                driveStraight.withVelocityX(0.75)
                             .withVelocityY(0.0)
                             .withHeadingPID(6, 0, 1)
                             .withTargetDirection(targetHeading[0])
            ).withTimeout(1.75),

            // Stop drivetrain
            drivetrain.applyRequest(() -> idle).withTimeout(0.05),

            // Wait before feeding
            Commands.waitSeconds(0.5),

            // Left bumper behavior
            Commands.parallel(
                mIntakeSpin_CMD,
                mRollerSpin_CMD,
                mTransfer_CMD,
                mTurretGoToTarget_CMD
            )
        )
        // Shooter runs entire auton
        .deadlineWith(mCloseShooterSpin_CMD);
    }
}