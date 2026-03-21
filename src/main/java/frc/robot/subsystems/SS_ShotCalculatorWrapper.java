package frc.robot.subsystems;

import custom.ShotCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_ShotCalculatorWrapper extends SubsystemBase {

  private static final Translation2d HUB_CENTER  = new Translation2d(8.23, 4.11);
  private static final Translation2d HUB_FORWARD = new Translation2d(1.0, 0.0);

  private final ShotCalculator m_calculator = new ShotCalculator();

  private ShotCalculator.LaunchParameters m_lastResult = ShotCalculator.LaunchParameters.INVALID;

  // Cache the pose used during the last calculate() so getTurretAngle() is consistent
  private Pose2d m_lastPose = new Pose2d();

  private final CommandSwerveDrivetrain m_swerve;

  public SS_ShotCalculatorWrapper(CommandSwerveDrivetrain swerve) {
    m_swerve = swerve;
  }

  @Override
  public void periodic() {
    var state = m_swerve.getState();

    m_lastPose = state.Pose;

    // state.Speeds is robot-relative per CTRE docs
    ChassisSpeeds robotSpeeds = state.Speeds;

    // Convert to field-relative for fieldVelocity param
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        robotSpeeds,
        m_lastPose.getRotation()
    );

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        m_lastPose,
        fieldSpeeds,
        robotSpeeds,
        HUB_CENTER,
        HUB_FORWARD,
        1.0
    );

    m_lastResult = m_calculator.calculate(inputs);
  }

  public ShotCalculator.LaunchParameters getResult() {
    return m_lastResult;
  }

  public boolean isReadyToShoot(double confidenceThreshold) {
    return m_lastResult.isValid() && m_lastResult.confidence() >= confidenceThreshold;
  }
}