package frc.robot.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import frc.robot.util.Conversions;
import frc.robot.util.Gearbox;
import frc.robot.util.SwerveModuleConstants;

public class Module {
    public String moduleName;
    public int moduleNumber;
    
    public TalonFX mDriveMotor;
    public TalonFX mAngleMotor;
    private CANCoder angleEncoder;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.drivekS, Constants.Swerve.drivekV, Constants.Swerve.drivekA);
    
    private double angleOffset;
    private double lastAngle;

    private boolean isDriveMotorInverted = false; // TODO: Check all
    private boolean isAngleMotorInverted = true;

    private Gearbox driveGearbox = new Gearbox(Swerve.driveGearboxRatio);
    private Gearbox angleGearbox = new Gearbox(Swerve.angleGearboxRatio);

    private double mWheelCircumference = Swerve.wheelCircumference;

    private double kMaxSpeed = Swerve.maxSpeed;

    public Module(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;

        /* CANcoder */
        angleEncoder = new CANCoder(moduleConstants.cancoderID, Swerve.CANivoreName);
        configAngleEncoder();

        /* Angle Motor */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Swerve.CANivoreName);
        configAngleMotor();

        /* Drive Motor */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Swerve.CANivoreName);
        configDriveMotor();

        angleEncoder.configMagnetOffset(moduleConstants.angleOffset);

        resetToAbsolute();

        lastAngle = getState().angle.getDegrees();


    }

    // sets individual module states and minimizes the change in heading
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    // for x-y translation
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // set the values of the input joystick
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity =
                    Conversions.MPSToFalcon(
                            desiredState.speedMetersPerSecond,
                            mWheelCircumference,
                            driveGearbox.getRatio());
            mDriveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    // for rotation
    private void setAngle(SwerveModuleState desiredState) {
        double angle =
                (Math.abs(desiredState.speedMetersPerSecond)
                                <= (kMaxSpeed * 0.01)) 
                        ? getAngle().getDegrees()
                        : desiredState.angle.getDegrees();

        mAngleMotor.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(angle, angleGearbox.getRatio()));
        lastAngle = angle;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(
                        mAngleMotor.getSelectedSensorPosition(), angleGearbox.getRatio()));
    }

    public void setAngleMotor(SwerveModuleState state) {
        SwerveModuleState desiredState = CTREModuleState.optimize(state, getState().angle);
        mAngleMotor.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(
                        desiredState.angle.getDegrees(), angleGearbox.getRatio()));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }


    public void resetToAbsolute() {
        double absolutePosition =
                Conversions.degreesToFalcon(
                        getCanCoder().getDegrees(), angleGearbox.getRatio());
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    public void setBrakeMode(boolean brake) {
        mDriveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    // Configs for motors & cancoder
    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.enableVoltageCompensation(true);
        mAngleMotor.setInverted(isAngleMotorInverted);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.configAllowableClosedloopError(
                0,
                Conversions.degreesToFalcon(
                        Constants.Swerve.allowableAngleError, angleGearbox.getRatio()));
        resetToAbsolute();
    }

    private void configDriveMotor() {
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(isDriveMotorInverted);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(
                        mDriveMotor.getSelectedSensorVelocity(),
                        mWheelCircumference,
                        driveGearbox.getRatio()),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(
                        mDriveMotor.getSelectedSensorPosition(),
                        mWheelCircumference,
                        driveGearbox.getRatio()),
                getAngle());
    }

    public int getModuleNumber() {
        return moduleNumber;
    }

    public TalonFX getDriveMotor() {
        return mDriveMotor;
    }

    public TalonFX getAngleMotor() {
        return mAngleMotor;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public String getName() {
        return moduleName;
    }



}
