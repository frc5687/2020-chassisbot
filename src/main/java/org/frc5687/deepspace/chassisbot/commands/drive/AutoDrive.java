package org.frc5687.deepspace.chassisbot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.commands.OutliersCommand;
import org.frc5687.deepspace.chassisbot.subsystems.SparkMaxDriveTrain;

public class AutoDrive extends OutliersCommand {
    private double _distance;
    private double _speed;
    private PIDController _distanceController;
    private PIDController _angleController;
    private double _distancePIDOut;
    private double _anglePIDOut;

    private boolean _usePID;
    private boolean _stopOnFinish;
    private double _angle;
    private String _stage;

    private SparkMaxDriveTrain _driveTrain;
    private AHRS _imu;

    private double kPdistance = 0.1; // .05;
    private double kIdistance = 0.000; // .001;
    private double kDdistance = 0.8; //.1;
    private double kTdistance = 1;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private double kTangle;


    /***
     * Drives for a set distance at a set speed.
     * @param distance Distance to drive
     * @param speed Speed to drive
     * @param usePID Whether to use pid or not
     * @param stopOnFinish Whether to stop the motors when we are done
     * @param angle The angle to drive, in degrees.  Pass 1000 to maintain robot's hading.
     */
    public AutoDrive(SparkMaxDriveTrain driveTrain, AHRS imu, double distance, double speed, boolean usePID, boolean stopOnFinish, double angle, String stage, double timeout) {
        super(timeout);
        requires(driveTrain);
        _speed = speed;
        _distance = distance;
        _usePID = usePID;
        _stopOnFinish = stopOnFinish;
        _angle = angle;
        _stage = stage;
        _driveTrain = driveTrain;
        _imu = imu;
    }

    @Override
    protected void initialize() {
        error("Starting AutoDrive");
        _driveTrain.resetDriveEncoders();

        _driveTrain.enableBrakeMode();
        if (_usePID) {
            metric("kP", kPdistance);
            metric("kI", kIdistance);
            metric("kD", kDdistance);
            metric("kT", kTdistance);

            _distanceController = new PIDController(kPdistance, kIdistance, kDdistance, _speed, _driveTrain, new DistanceListener(), 0.01);
            _distanceController.setAbsoluteTolerance(kTdistance);
            _distanceController.setOutputRange(-_speed, _speed);
            _distanceController.setSetpoint(_driveTrain.getDistance() + _distance);
            _distanceController.enable();
        }

        _angleController = new PIDController(kPangle, kIangle, kDangle, _imu, new AngleListener(), 0.01);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        double maxSpeed = _speed * Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE;
        metric("angleMaxSpeed", maxSpeed);
        metric("setPoint", _driveTrain.getYaw());
        _angleController.setOutputRange(-maxSpeed, maxSpeed);
        _angleController.setContinuous();

        // If an angle is supplied, use that as our setpoint.  Otherwise get the current heading and stick to it!
        _angleController.setSetpoint(Math.abs(_angle) > 180 ? _driveTrain.getYaw(): _angle);
        _angleController.enable();

        info("Auto Drive initialized: " + (_stage ==null?"": _stage));
    }

    @Override
    protected void execute() {
        super.execute();
        double baseSpeed = _usePID ? _distancePIDOut : (_distance < 0 ? -_speed : _speed);
        metric("LeftSpeed", baseSpeed + _anglePIDOut);
        metric("RightSpeed", baseSpeed - _anglePIDOut);
        _driveTrain.setPower(baseSpeed + _anglePIDOut , baseSpeed - _anglePIDOut, true); // positive output is clockwise
        metric("onTarget", _distanceController == null ? false : _distanceController.onTarget());
        metric("Angle", _driveTrain.getYaw());
        metric("Distance", _driveTrain.pidGet());
        metric("turnPID", _anglePIDOut);
        metric("distancePID", _distancePIDOut);
    }

    @Override
    protected boolean isFinished() {
        if (_usePID) {
            if (_distanceController.onTarget()) {
                error("AutoDrive stopped at " + _driveTrain.getDistance());
                return true;
            }
        } else {
            if (_distance == 0) {
                info("AutoDrive nopid complete at " + _driveTrain.getDistance() + " inches");
                return true;
            } else if (_distance < 0 && _driveTrain.getDistance() < _distance) {
                info("AutoDrive nopid complete at " + _driveTrain.getDistance() + " inches");
                if (_stopOnFinish) {
                    _driveTrain.setPower(0, 0, true);
                }
                return true;
            } else if (_distance > 0 && _driveTrain.getDistance() > _distance)  {
                info("AutoDrive nopid complete at " + _driveTrain.getDistance() + " inches");
                if (_stopOnFinish) {
                    _driveTrain.setPower(0, 0, true);
                }
                return true;
            }
        }
        return false;
    }



    @Override
    protected void end() {
        super.end();
        if (isTimedOut()) {
            error("AutoDrive timed out (" + _driveTrain.getDistance() + ", " + (_driveTrain.getYaw() - _angleController.getSetpoint()) + ") " + (_stage ==null?"": _stage));
        } else {
            error("AutoDrive Finished (" + _driveTrain.getDistance() + ", " + (_driveTrain.getYaw() - _angleController.getSetpoint()) + ") " + (_stage == null ? "" : _stage));
        }
        if (_distanceController !=null) {
            _distanceController.disable();
        }
        if (_stopOnFinish) {
            error("AutoDrive stopping at " + _driveTrain.getDistance());
            _driveTrain.setPower(0, 0, true);
        }
        _angleController.disable();
    }


    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }
    }

    private class DistanceListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _distancePIDOut = output;
            }
        }
    }


}

