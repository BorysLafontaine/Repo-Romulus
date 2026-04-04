package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import org.littletonrobotics.junction.Logger;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {

    // =========================
    // INPUT FILTERING
    // =========================
    private final SlewRateLimiter xLimiter     = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter     = new SlewRateLimiter(3);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(6);

    private final double MaxSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // =========================
    // SUBSYSTEMS
    // =========================
    private final LEDSubsystem mLedSubsystem = new LEDSubsystem();

    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final VisionSubsystem vision = new VisionSubsystem();

    private final SS_Shooter       mShooter      = new SS_Shooter();
    private final SS_TurretFixed   mTurretFixed  = new SS_TurretFixed();
    private final SS_Intake        mIntake       = new SS_Intake();
    private final SS_IntakeMotors  mIntakeMotors = new SS_IntakeMotors();
    private final SS_Transfer      mTransfer     = new SS_Transfer();
    private final SS_Rollers       mRollers      = new SS_Rollers();

    // =========================
    // COMMANDS
    // =========================
    private final ShooterSpin_CMD        mShooterSpin_CMD      = new ShooterSpin_CMD(mShooter);
    private final CloseShooterSpin_CMD   mCloseShooterSpin_CMD = new CloseShooterSpin_CMD(mShooter);
    private final FarShooterSpin_CMD     mFarShooterSpin_CMD   = new FarShooterSpin_CMD(mShooter);

    private final TurretGoToTarget_CMD mTurretGoToTarget_CMD =
        new TurretGoToTarget_CMD(mTurretFixed);

    private final toggle_intake_CMD mtoggle_intake_CMD =
        new toggle_intake_CMD(mIntake);

    private final IntakeSpin_CMD mIntakeSpin_CMD =
        new IntakeSpin_CMD(mIntakeMotors);

    private final ReverseIntakeSpin_CMD mReverseIntakeSpin_CMD =
        new ReverseIntakeSpin_CMD(mIntakeMotors);

    private final Transfer_CMD mTransfer_CMD =
        new Transfer_CMD(mTransfer);

    private final ReverseTransfer_CMD mReverseTransfer_CMD =
        new ReverseTransfer_CMD(mTransfer);

    private final RollerSpin_CMD mRollerSpin_CMD =
        new RollerSpin_CMD(mRollers);

    private final ReverseRollerSpin_CMD mReverseRollerSpin_CMD =
        new ReverseRollerSpin_CMD(mRollers);

    private final LeftBumperGroup_CMD mLeftBumperGroup =
        new LeftBumperGroup_CMD(mIntakeMotors, mRollers, mTransfer);

    // =========================
    // CONTROLLER
    // =========================
    private final CommandXboxController joystick =
        new CommandXboxController(0);

    // =========================
    // DRIVE REQUESTS
    // =========================
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(0.05)
            .withRotationalDeadband(0.05)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake =
        new SwerveRequest.SwerveDriveBrake();

    // =========================
    // CONSTRUCTOR
    // =========================
    public RobotContainer() {
        configureBindings();
    }

    // =========================
    // BINDINGS
    // =========================
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double vx    = xLimiter.calculate(-joystick.getLeftY())  * MaxSpeed;
                double vy    = yLimiter.calculate(-joystick.getLeftX())  * MaxSpeed;
                double omega = omegaLimiter.calculate(-joystick.getRightX()) * MaxAngularRate;

                Logger.recordOutput("Drive/Vx",    vx);
                Logger.recordOutput("Drive/Vy",    vy);
                Logger.recordOutput("Drive/Omega", omega);

                return drive.withVelocityX(vx)
                            .withVelocityY(vy)
                            .withRotationalRate(omega);
            })
        );

        // =========================
        // VISION FUSION LOOP
        // FIX: was calling the 2-arg addVisionMeasurement, which bypasses
        //      the custom override in CommandSwerveDrivetrain entirely.
        //      Now passes std devs explicitly to hit the 3-arg override.
        // =========================
        drivetrain.registerTelemetry(state -> {

            var est = vision.getLatestEstimate();

            if (est.isPresent()) {

                var pose      = est.get().estimatedPose.toPose2d();
                var targets   = est.get().targetsUsed;

                drivetrain.resetPose(pose);

                Logger.recordOutput("Vision/Pose",     pose);
                Logger.recordOutput("Vision/TagCount", targets.size());
            }
        });

        // =========================
        // DISABLED MODE
        // =========================
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // =========================
        // BUTTONS
        // =========================
        joystick.x().toggleOnTrue(mShooterSpin_CMD);
        joystick.povDown().toggleOnTrue(mCloseShooterSpin_CMD);
        joystick.povUp().toggleOnTrue(mFarShooterSpin_CMD);

        joystick.povRight().toggleOnTrue(mTurretGoToTarget_CMD);

        joystick.y().onTrue(mtoggle_intake_CMD);

        joystick.rightBumper().toggleOnTrue(mIntakeSpin_CMD);

        joystick.leftBumper().whileTrue(mLeftBumperGroup);

        joystick.b().whileTrue(mReverseIntakeSpin_CMD);
        joystick.b().whileTrue(mReverseRollerSpin_CMD);
        joystick.b().whileTrue(mReverseTransfer_CMD);

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.povLeft().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        mLedSubsystem.updateLEDs();
    }

    // =========================
    // AUTON
    // =========================
    public Command getAutonomousCommand() {

        final var idle = new SwerveRequest.Idle();
        final Rotation2d[] targetHeading = new Rotation2d[1];

        return Commands.sequence(

            drivetrain.runOnce(() ->
                drivetrain.seedFieldCentric(Rotation2d.kZero)
            ),

            Commands.runOnce(() ->
                targetHeading[0] = drivetrain.getState().Pose.getRotation()
            ),

            drivetrain.applyRequest(() ->
                drive.withVelocityX(1.0)
                     .withVelocityY(0.0)
                     .withRotationalRate(0.0)
            ).withTimeout(2.0),

            drivetrain.applyRequest(() -> idle).withTimeout(0.05),

            Commands.waitSeconds(0.5),

            mLeftBumperGroup
        )
        .deadlineWith(mCloseShooterSpin_CMD);
    }
}