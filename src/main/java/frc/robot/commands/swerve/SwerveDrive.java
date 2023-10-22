// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends CommandBase {
  private final Swerve mDrive = Swerve.getInstance();
  private final DoubleSupplier mX; // strafe
  private final DoubleSupplier mY; // translation
  private final DoubleSupplier mRotation; // rotation
  private final BooleanSupplier fieldOriented; // robot centric or field centric

  private final BooleanSupplier mRun; // higher speed
  private final BooleanSupplier mSlow; // more controlled/slower

  private final SlewRateLimiter xLimiter, yLimiter, rotationLimiter;
  private double driveScalarValue = Constants.Swerve.drivePowerScalar;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(
      DoubleSupplier translation,
      DoubleSupplier strafe,
      DoubleSupplier rotation,
      BooleanSupplier mRun,
      BooleanSupplier mSlow,
      BooleanSupplier fieldOriented) {

    mX = strafe;
    mY = translation;
    mRotation = rotation;
    this.mRun = mRun;
    this.mSlow = mSlow;
    this.fieldOriented = fieldOriented;

    xLimiter = new SlewRateLimiter(Constants.Swerve.driveSlewRateLimitX);
    yLimiter = new SlewRateLimiter(Constants.Swerve.driveSlewRateLimitY);
    rotationLimiter = new SlewRateLimiter(Constants.Swerve.driveSlewRateLimitRot);
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double scalar = mRun.getAsBoolean() ? 1 : driveScalarValue;
    if (mSlow.getAsBoolean()) {
      scalar = 0.3;
    } else if (mRun.getAsBoolean()) {
      scalar = 1;
    } else {
      scalar = driveScalarValue;
    }

    double xSpeed = xLimiter.calculate(inputTransform(mX.getAsDouble()) * Constants.Swerve.maxSpeed) * scalar;
    double ySpeed = yLimiter.calculate(inputTransform(mY.getAsDouble()) * Constants.Swerve.maxSpeed) * scalar;
    double rotation = rotationLimiter.calculate(inputTransform(mRotation.getAsDouble()) * Constants.Swerve.maxAngularSpeedRadPerSec) * scalar;
    mDrive.drive(new Translation2d(ySpeed, xSpeed), rotation, fieldOriented.getAsBoolean(), true); // TODO: DOUBLE CHECK FIELD ORIENTATION HERE
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
  private double inputTransform(double input) {
    if (input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
  }
}
