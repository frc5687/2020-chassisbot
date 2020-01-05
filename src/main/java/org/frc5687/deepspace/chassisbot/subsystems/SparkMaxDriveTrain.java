package org.frc5687.deepspace.chassisbot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;
import org.frc5687.deepspace.chassisbot.commands.Drive;
import edu.wpi.first.wpilibj.*;
import org.frc5687.deepspace.chassisbot.utils.Limelight;


import static org.frc5687.deepspace.chassisbot.Constants.DriveTrain.CREEP_FACTOR;
import static org.frc5687.deepspace.chassisbot.utils.Helpers.applySensitivityFactor;
import static org.frc5687.deepspace.chassisbot.utils.Helpers.limit;

public class SparkMaxDriveTrain extends OutliersSubsystem implements PIDSource {
    private CANSparkMax _frontLeftSpark;
    private CANSparkMax _frontRightSpark;
    private CANSparkMax _backLeftSpark;
    private CANSparkMax _backRightSpark;

    private double _oldLeftSpeedFront;
    private double _oldLeftSpeedBack;
    private double _oldRightSpeedFront;
    private double _oldRightSpeedBack;
    private boolean _isPaused = false;

    private Encoder _leftEncoder;
    private Encoder _rightEncoder;

    private OI _oi;
    private AHRS _imu;
    private Limelight _limelight;
    private Shifter _shifter;
    private Robot _robot;

    public SparkMaxDriveTrain (Robot robot) {
        info("Constructing SparkMaxDriveTrain class.");
        _oi = robot.getOI();
        _imu = robot.getIMU();
        _shifter = robot.getShifter();
        _limelight = robot.getLimelight();

        _robot = robot;


        _frontLeftSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_FRONT_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _backLeftSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_BACK_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _frontRightSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_FRONT_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _backRightSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_BACK_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);

        _frontLeftSpark.restoreFactoryDefaults();
        _backLeftSpark.restoreFactoryDefaults();
        _frontRightSpark.restoreFactoryDefaults();
        _backRightSpark.restoreFactoryDefaults();


        _frontLeftSpark.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _backLeftSpark.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _frontRightSpark.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _backRightSpark.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);

        _frontLeftSpark.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _backLeftSpark.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _frontRightSpark.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
        _backRightSpark.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);

        _frontLeftSpark.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
        _backLeftSpark.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
        _frontRightSpark.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
        _backRightSpark.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);

        _frontLeftSpark.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
        _backLeftSpark.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
        _frontRightSpark.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
        _backRightSpark.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);

        _frontLeftSpark.setInverted(Constants.DriveTrain.LEFT_FRONT_MOTOR_INVERTED);
        _backLeftSpark.setInverted(Constants.DriveTrain.LEFT_BACK_MOTOR_INVERTED);
        _frontRightSpark.setInverted(Constants.DriveTrain.RIGHT_FRONT_MOTOR_INVERTED);
        _backRightSpark.setInverted(Constants.DriveTrain.RIGHT_BACK_MOTOR_INVERTED);

        _leftEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_A, RobotMap.DIO.DRIVE_LEFT_B);
        _rightEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_A, RobotMap.DIO.DRIVE_RIGHT_B);
        _leftEncoder.setDistancePerPulse(Constants.DriveTrain.LEFT_DISTANCE_PER_PULSE);
        _rightEncoder.setDistancePerPulse(Constants.DriveTrain.RIGHT_DISTANCE_PER_PULSE);
        resetDriveEncoders();
    }

    public void enableBrakeMode() {
        _frontLeftSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _backLeftSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _frontRightSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _frontRightSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void disableBrakeMode() {
        _frontLeftSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _backLeftSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _frontRightSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _backRightSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }


        @Override
    public void updateDashboard() {
        metric("Ticks/Left", _leftEncoder.get());
        metric("Ticks/Right", _rightEncoder.get());
        metric("MagDistance/Left", _leftEncoder.getDistance());
        metric("MagDistance/Right", _rightEncoder.getDistance());
        metric("Distance/Left", getLeftDistance());
        metric("Distance/Right", getRightDistance());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(this, _imu, _oi,  _limelight, _robot.getPoseTracker()));
    }

    public void cheesyDrive(double speed, double rotation, boolean creep, boolean override) {
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed, 1);
        //Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation, 1);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed < Constants.DriveTrain.DEADBAND && speed > -Constants.DriveTrain.DEADBAND) {
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);
            }
            if (creep) {
                //metric("Rot/Creep", creep);
                rotation = rotation * CREEP_FACTOR;
            } else {
                rotation = rotation * 0.8;
            }

//            metric("Rot/Transformed", rotation);
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
//            metric("Rot/LeftMotor", leftMotorOutput);
//            metric("Rot/RightMotor", rightMotorOutput);
        } else {
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            metric("Str/Raw", speed);
            speed = Math.copySign(applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY), speed);
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.TURNING_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.TURNING_SENSITIVITY_LOW_GEAR);
            }
            metric("Str/Trans", speed);
            rotation = applySensitivityFactor(rotation, Constants.DriveTrain.TURNING_SENSITIVITY);
            double delta = override ? rotation : rotation * Math.abs(speed);
            if (override) {
                // speed = Math.copySign(limit(Math.abs(speed), 1-Math.abs(delta)), speed);

                if (speed + Math.abs(delta) > 1) {
                    speed = 1 - Math.abs(delta);
                }

                if (speed - Math.abs(delta) < -1) {
                    speed = -1 + Math.abs(delta);
                }
            }
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
            metric("Str/LeftMotor", leftMotorOutput);
            metric("Str/RightMotor", rightMotorOutput);
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }

    public float getYaw() {
        return _imu.getYaw();
    }

    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        _frontLeftSpark.set(leftSpeed);
        _backLeftSpark.set (leftSpeed);
        _frontRightSpark.set(rightSpeed);
        _backRightSpark.set (rightSpeed);
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }

    public void pauseMotors() {
        _oldLeftSpeedFront = _frontLeftSpark.get();
        _oldLeftSpeedBack = _backLeftSpark.get();
        _oldRightSpeedFront = _frontRightSpark.get();
        _oldRightSpeedBack = _backRightSpark.get();
        _frontLeftSpark.set(0);
        _backLeftSpark.set(0);
        _frontRightSpark.set(0);
        _backRightSpark.set(0);
        _isPaused = true;
    }

    public void resumeMotors() {
        _frontLeftSpark.set(_oldLeftSpeedFront);
        _backLeftSpark.set(_oldLeftSpeedBack);
        _frontRightSpark.set(_oldRightSpeedFront);
        _backRightSpark.set(_oldRightSpeedBack);
        _isPaused = false;
    }

    public double getLeftDistance() {
        return getLeftTicks() * Constants.DriveTrain.LEFT_DISTANCE_PER_PULSE;
    }

    public double getRightDistance() {
        return getRightTicks() * Constants.DriveTrain.RIGHT_DISTANCE_PER_PULSE;
    }

    public double getLeftTicks() {
        return _leftEncoder.get();
    }

    public double getRightTicks() {
        return _rightEncoder.get();
    }

    public double getDistance() {
        if (Math.abs(getRightTicks())<10) {
            return getLeftDistance();
        }
        if (Math.abs(getLeftTicks())<10) {
            return getRightDistance();
        }
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public void resetDriveEncoders() {
        _leftEncoder.reset();
        _rightEncoder.reset();
    }

    @Override
    public double pidGet() {
        return getDistance();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    public double getLeftPower() {
        return _frontLeftSpark.get();
    }

    public double getRightPower() {
        return _frontRightSpark.get();
    }




}


