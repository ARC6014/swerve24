// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Swerve {

        public static final String CANivoreName = "rio";
        public static final int pigeonId = 50;
        public static final boolean invertGyro = false; // * CCW+

        public static final double trackWidth = 0.728; // TODO: Config
        public static final double wheelBase = 0.728; // TODO: Config
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        public static final SwerveDriveKinematics swerveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        // Drive specs
        public static final double angleGearboxRatio = 22.93; // TODO: Config
        public static final double driveGearboxRatio = 6.59340659; // TODO: Config
        

        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        public static final double maxSpeed = 5;
        public static final double maxTransSpeedMetersPerSecond = 3.3; // translation speed (x/y)
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI; // angular speed (omega)
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2); // angular acceleration


        public static final double drivePowerScalar = 0.55; 
        public static final double driveSlewRateLimitX = 7;
        public static final double driveSlewRateLimitY = 7;
        public static final double driveSlewRateLimitRot = 12;

        // Limits for drive&rotation
        public static final double openLoopRamp = 0;
        public static final double closedLoopRamp = 0;

        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 35;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.2;
        public static final boolean driveEnableCurrentLimit = true;

        public static final double allowableAngleError = 0.10;

        // PID and Feedforward
        public static final double drivekP = 0.05;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.016;
        public static final double drivekV = 0.19;
        public static final double drivekA = 0.0;

        public static final double anglekP = 0.27;
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        public static final double snapkP = 2.5;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.01;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 01;
            public static final double angleOffset = -206.63 - 45 - 17 + 180;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 16;
            public static final int angleMotorID = 17;
            public static final int canCoderID = 02;
            public static final double angleOffset = -152.06 - 12.48;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 15;
            public static final int canCoderID = 03;
            public static final double angleOffset = -22.06 + 180 -115;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 04;
            public static final double angleOffset = -1.49 + 109.4;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

    }
}
