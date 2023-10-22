// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.swerve.Module;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  // Swerve modules go:
  // 0 1
  // 2 3
  private static Swerve mInstance;
  private static boolean m_isFieldOriented = true;

  /*
   * Triggers when disabled and sets motors to coast so that we can turn them
   * easily
   */
  private final Trigger brakeModeTrigger;
  private final StartEndCommand brakeModeCommand;

  /* Stuff for swerve config */
  public Module[] mSwerveMods; // all swerve modules
  public WPI_Pigeon2 gyro;
  private final SwerveDrivePoseEstimator estimator;

  private final Field2d m_field2d = new Field2d();

  // TODO: NOTE: ADD SNAP PID IF NEEDED

  /** Creates a new Swerve. */
  public Swerve() {
    gyro = new WPI_Pigeon2(Constants.Swerve.pigeonId, Constants.Swerve.CANivoreName);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods = new Module[] {
        new Module("FL", 0, Constants.Swerve.Mod0.constants),
        new Module("FR", 1, Constants.Swerve.Mod1.constants),
        new Module("BL", 2, Constants.Swerve.Mod2.constants),
        new Module("BR", 3, Constants.Swerve.Mod3.constants)
    };

    Timer.delay(1.0);
    resetToAbsolute();

    estimator = new SwerveDrivePoseEstimator(
        frc.robot.Constants.Swerve.swerveKinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d());
    SmartDashboard.putData(m_field2d);
    brakeModeTrigger = new Trigger(RobotState::isEnabled);
    brakeModeCommand = new StartEndCommand(() -> {
      for (Module mod : mSwerveMods) {
        mod.setBrakeMode(true);
      }
    }, () -> {
      Timer.delay(1.5);
      for (Module mod : mSwerveMods) {
        mod.setBrakeMode(false);
      }
    });
  }

  public static Swerve getInstance() {
    if (mInstance == null) {
      mInstance = new Swerve();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimator.update(getRotation2d(), getModulePositions());
    m_field2d.setRobotPose(getPose());

    brakeModeTrigger.whileTrue(brakeModeCommand);
  }

  /* Open Loop Swerve drive */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                estimator.getEstimatedPosition().getRotation())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (Module mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Closed Loop Swerve drive */
  /* Used by PPSwerveControllerCommand in Auto */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (Module mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {

    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (Module mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (Module mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    estimator.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void zeroGyro() {
    gyro.zeroGyroBiasNow();
    gyro.setYaw(0);
  }

  public void resetToAbsolute() {
    for (Module mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(
        Math.IEEEremainder(gyro.getAngle(), 360)
            * (Constants.Swerve.invertGyro ? -1.0 : 1.0));
  }

  public boolean getIsFieldOriented() {
    return !m_isFieldOriented;
  }

  public void setFieldOriented() {
    m_isFieldOriented = true;
  }

  public void setRobotOriented() {
    m_isFieldOriented = false;
  }
}
