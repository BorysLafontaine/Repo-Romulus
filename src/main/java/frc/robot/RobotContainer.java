package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(4);

    private final double MaxSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final double MaxAngularRate =
        RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // =========================
    // SUBSYSTEMS
    // =========================
    public final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain();

    private final VisionSubsystem vision = new VisionSubsystem();

    private final SS_Shooter   mShooter   = new SS_Shooter();
    private final SS_TurretAim mTurretAim = new SS_TurretAim();
    private final SS_Intake        mIntake       = new SS_Intake();
    private final SS_IntakeMotors  mIntakeMotors = new SS_IntakeMotors();
    private final SS_Transfer      mTransfer     = new SS_Transfer();
    private final SS_Rollers       mRollers      = new SS_Rollers();

    // =========================
    // COMMANDS
    // =========================
    private final ShooterSpin_CMD     mShooterSpin_CMD     = new ShooterSpin_CMD(mShooter, drivetrain);
    private final TurretResetPose_CMD mTurretResetPose_CMD = new TurretResetPose_CMD(mTurretAim);

    // Hub aiming — robot pose is sourced live from drivetrain odometry
    private final TurretAimAtHub_CMD mTurretAimAtHub_CMD =
        new TurretAimAtHub_CMD(mTurretAim, drivetrain);

    private final toggle_intake_CMD mtoggle_intake_CMD =
        new toggle_intake_CMD(mIntake);

    private final IntakeSpin_CMD mIntakeSpin_CMD =
        new IntakeSpin_CMD(mIntakeMotors);

    private final ReverseIntakeSpin_CMD mReverseIntakeSpin_CMD =
        new ReverseIntakeSpin_CMD(mIntakeMotors);

    private final ReverseTransfer_CMD mReverseTransfer_CMD =
        new ReverseTransfer_CMD(mTransfer);

    private final ReverseRollerSpin_CMD mReverseRollerSpin_CMD =
        new ReverseRollerSpin_CMD(mRollers);

    private final LeftBumperGroup_CMD mLeftBumperGroup =
        new LeftBumperGroup_CMD(mIntakeMotors, mRollers, mTransfer);

    // =========================
    // AUTO CHOOSER
    // =========================
    private final SendableChooser<Command> autoChooser;

    // =========================
    // CONTROLLER
    // =========================
    private final CommandPS5Controller joystick =
        new CommandPS5Controller(0);

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
        registerNamedCommands();
        configureBindings();

        // Build chooser AFTER configureAutoBuilder() has been called
        // (which happens inside the drivetrain constructor)
        autoChooser = AutoBuilder.buildAutoChooser();

        // "Auto Chooser" is the key Elastic's SendableChooser widget looks for
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // =========================
    // NAMED COMMANDS
    // These string keys must exactly match what you use in PathPlanner's GUI.
    // =========================
    private void registerNamedCommands() {

        // Shooter
        NamedCommands.registerCommand("ShooterSpin",      new ShooterSpin_CMD(mShooter, drivetrain));
        NamedCommands.registerCommand("CloseShooterSpin", new ShooterSpin_CMD(mShooter, drivetrain));
        NamedCommands.registerCommand("FarShooterSpin",   new ShooterSpin_CMD(mShooter, drivetrain));

        // Intake
        NamedCommands.registerCommand("DeployIntake",     Commands.runOnce(mIntake::deploy, mIntake));
        NamedCommands.registerCommand("RetractIntake",    Commands.runOnce(mIntake::retract, mIntake));
        NamedCommands.registerCommand("IntakeSpin",       new IntakeSpin_CMD(mIntakeMotors));
        NamedCommands.registerCommand("ReverseIntake",    new ReverseIntakeSpin_CMD(mIntakeMotors));

        // Groups
        NamedCommands.registerCommand("Collect",
            new LeftBumperGroup_CMD(mIntakeMotors, mRollers, mTransfer).withTimeout(4));
        NamedCommands.registerCommand("Transfer",
            new LeftBumperGroup_CMD(mIntakeMotors, mRollers, mTransfer).withTimeout(4));

        // Turret
        NamedCommands.registerCommand("TurretAimHub",
            new TurretAimAtHub_CMD(mTurretAim, drivetrain));

        // Aim + shoot in one shot — use this in PathPlanner to aim, spin up, and fire
        NamedCommands.registerCommand("AimAndShoot",
            new AutonAimAndShoot_CMD(mTurretAim, mShooter, drivetrain));
    }

    // =========================
    // BINDINGS
    // =========================
    private void configureBindings() {

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {

                double vx    = xLimiter.calculate(-joystick.getLeftY())      * MaxSpeed;
                double vy    = yLimiter.calculate(-joystick.getLeftX())      * MaxSpeed;
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
        // getAllEstimates() returns one entry per camera that passed all filters.
        // addVisionMeasurements() hard-resets pose every cycle when tags are visible.
        // Falls back to pure odometry when no tags are seen.
        // =========================
        drivetrain.registerTelemetry(state -> {

            // Snapshot pure wheel odometry BEFORE vision resets the pose
            drivetrain.snapshotOdometryPose(state.Pose);

            // Keep estimators aligned to current odometry so single-tag picks the right solution
            vision.setReferencePose(state.Pose);

            var estimates = vision.getAllEstimates();

            drivetrain.addVisionMeasurements(estimates);

            estimates.forEach(e -> {
                Logger.recordOutput("Vision/Pose",     e.pose());
                Logger.recordOutput("Vision/TagCount", e.tagCount());
                Logger.recordOutput("Vision/AvgDist",  e.avgDistanceMeters());
            });
        });

        // =========================
        // AUTON — aim + shooter always running for the full autonomous period
        // Transfer is NOT included; trigger it separately via a named command.
        // =========================
        RobotModeTriggers.autonomous().whileTrue(
            new AutonAimAndShoot_CMD(mTurretAim, mShooter, drivetrain)
        );

        // =========================
        // DISABLED MODE
        // =========================
        final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // =========================
        // AUTO POSE RESET ON TELEOP ENABLE
        // When teleop begins, immediately hard-reset from vision if a
        // high-confidence multi-tag estimate is available.  This corrects
        // any odometry drift that built up during auto or while disabled.
        // =========================
        // Teleop enable: attempt immediate pose snap from vision.
        // If no tags are visible yet, the vision loop above will snap it on the
        // first cycle where a tag is seen (no separate retry needed).
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() ->
            vision.getHighConfidenceEstimate()
                  .ifPresent(drivetrain::hardResetPoseFromVision)
        ));

        // =========================
        // BUTTONS
        // =========================
        joystick.square().toggleOnTrue(mShooterSpin_CMD);

        // POV right: toggle auto aim — on = aiming, off = motor coasts (disabled)
        joystick.povRight().toggleOnTrue(mTurretAimAtHub_CMD);

        // POV up: zero encoder at current position (interrupts aim if running)
        joystick.povUp().onTrue(mTurretResetPose_CMD);

        // POV left: reset field-centric heading
        joystick.povLeft().onTrue(
            drivetrain.runOnce(drivetrain::seedFieldCentric)
        );

        joystick.triangle().onTrue(mtoggle_intake_CMD);

        joystick.R1().toggleOnTrue(mIntakeSpin_CMD);

        joystick.L1().whileTrue(mLeftBumperGroup);

        joystick.circle().whileTrue(mReverseIntakeSpin_CMD);
        joystick.circle().whileTrue(mReverseRollerSpin_CMD);
        joystick.circle().whileTrue(mReverseTransfer_CMD);

        joystick.cross().whileTrue(drivetrain.applyRequest(() -> brake));

        // =========================
        // HARD POSE RESET FROM VISION
        // Options button: instantly snap odometry to the camera's pose estimate
        // when at least 2 tags are visible with high confidence.
        // =========================
        joystick.options().onTrue(Commands.runOnce(() ->
            vision.getHighConfidenceEstimate()
                  .ifPresent(drivetrain::hardResetPoseFromVision)
        ));

    }

    // =========================
    // AUTON
    // =========================
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}