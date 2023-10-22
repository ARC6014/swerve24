// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.swerve.Module;
import frc.robot.util.Conversions;
import frc.robot.subsystems.Swerve;

public class LockSwerve extends CommandBase {
    private Swerve m_swerve;
    private SwerveModuleState[] states;

    /** Creates a new SetSwerveX. */
    public LockSwerve(Swerve swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        states =
                Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));

        for (Module mod : m_swerve.mSwerveMods) {
            states[mod.moduleNumber].angle =
                    new Rotation2d(
                            states[mod.moduleNumber].angle.getRadians() + Units.degreesToRadians(90.0));

            mod.mAngleMotor.configAllowableClosedloopError(
                    0, Conversions.degreesToFalcon(1.0, Constants.Swerve.angleGearboxRatio));
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        for (Module mod : m_swerve.mSwerveMods) {
            mod.setAngleMotor(new SwerveModuleState(0.0, states[mod.moduleNumber].angle));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        for (Module mod : m_swerve.mSwerveMods) {
            mod.mAngleMotor.configAllowableClosedloopError(
                    0,
                    Conversions.degreesToFalcon(
                            Constants.Swerve.allowableAngleError, Constants.Swerve.angleGearboxRatio));
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
