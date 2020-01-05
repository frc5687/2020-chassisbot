package org.frc5687.deepspace.chassisbot.subsystems;


import edu.wpi.first.wpilibj.*;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;
import org.frc5687.deepspace.chassisbot.commands.Drive;

import static org.frc5687.deepspace.chassisbot.Constants.DriveTrain.CREEP_FACTOR;
import static org.frc5687.deepspace.chassisbot.utils.Helpers.applySensitivityFactor;
import static org.frc5687.deepspace.chassisbot.utils.Helpers.limit;

public class VictorSPDriveTrain extends OutliersSubsystem {
    private VictorSP _frontLeftVictor;
    private VictorSP _backLeftVictor;
    private VictorSP _frontRightVictor;
    private VictorSP _backRightVictor;

    private OI _oi;

    public VictorSPDriveTrain(Robot robot) {
        info("Constructing VictorSPDriveTrain class.");
        _oi = robot.getOI();

        _frontLeftVictor = new VictorSP(RobotMap.PWM.LEFT_FRONT_DRIVE_MOTOR);
        _backLeftVictor = new VictorSP(RobotMap.PWM.LEFT_BACK_DRIVE_MOTOR);
        _frontRightVictor = new VictorSP(RobotMap.PWM.RIGHT_FRONT_DRIVE_MOTOR);
        _backRightVictor = new VictorSP(RobotMap.PWM.RIGHT_BACK_DRIVE_MOTOR);

        _frontLeftVictor.setInverted(Constants.DriveTrain.LEFT_FRONT_MOTOR_INVERTED);
        _backLeftVictor.setInverted(Constants.DriveTrain.LEFT_BACK_MOTOR_INVERTED);
        _frontRightVictor.setInverted(Constants.DriveTrain.RIGHT_FRONT_MOTOR_INVERTED);
        _backRightVictor.setInverted(Constants.DriveTrain.RIGHT_BACK_MOTOR_INVERTED);


    }


    @Override
    public void updateDashboard() {

    }

    @Override
    protected void initDefaultCommand() {
       // setDefaultCommand(new Drive(this, _oi));
    }

    public void cheesyDrive(double speed, double rotation, boolean creep) {
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed, 1);
        //Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation, 1);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed < Constants.DriveTrain.DEADBAND && speed > -Constants.DriveTrain.DEADBAND) {
            metric("Rot/Raw", rotation);
            rotation = applySensitivityFactor(rotation, Constants.DriveTrain.ROTATION_SENSITIVITY);
            if (creep) {
                metric("Rot/Creep", creep);
                rotation = rotation * CREEP_FACTOR;
            }

            metric("Rot/Transformed", rotation);
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
            metric("Rot/LeftMotor", leftMotorOutput);
            metric("Rot/RightMotor", rightMotorOutput);
        } else {
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            metric("Str/Raw", speed);
            speed = Math.copySign(applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY), speed);
            metric("Str/Trans", speed);
            rotation = applySensitivityFactor(rotation, Constants.DriveTrain.TURNING_SENSITIVITY);
            double delta = rotation * Math.abs(speed);
            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
            metric("Str/LeftMotor", leftMotorOutput);
            metric("Str/RightMotor", rightMotorOutput);
        }

        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }

    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
        _frontLeftVictor.set(leftSpeed);
        _backLeftVictor.set (leftSpeed);
        _frontRightVictor.set(rightSpeed);
        _backRightVictor.set (rightSpeed);
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }

    public double getLeftPower() {
        return _frontLeftVictor.get();
    }

    public double getRightPower() {
        return _frontRightVictor.get();
    }


}
