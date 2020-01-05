package org.frc5687.deepspace.chassisbot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.SparkMaxDriveTrain;
import org.frc5687.deepspace.chassisbot.subsystems.VictorSPDriveTrain;
import org.frc5687.deepspace.chassisbot.utils.BasicPose;
import org.frc5687.deepspace.chassisbot.utils.Helpers;
import org.frc5687.deepspace.chassisbot.utils.Limelight;
import org.frc5687.deepspace.chassisbot.utils.PoseTracker;

import static org.frc5687.deepspace.chassisbot.Constants.Auto.Align.STEER_K;

public class Drive extends OutliersCommand {

    private OI _oi;
    private VictorSPDriveTrain _driveTrainVictor;
    private SparkMaxDriveTrain _driveTrainSpark;
    private AHRS _imu;
    private Limelight _limelight;
    private PoseTracker _poseTracker;

    private PIDController _angleController;

    private double _anglePIDOut;
    private double _angle;
    private double _turnSpeed;
    private boolean _targetSighted;
    private long _lockEnd;
    private DriveState _driveState = DriveState.normal;

    private long _seekMax;
    private double _stickyLimit;
    private boolean _lockout = false;

    private double _mediumZone;
    private double _slowZone;

    private double _slowSpeed;
    private double _mediumSpeed;

    private int garbageCount = 0;

    public Drive(SparkMaxDriveTrain driveTrain, AHRS imu, OI oi, Limelight limelight, PoseTracker poseTracker) {
        _driveTrainSpark = driveTrain;
        _imu = imu;
        _oi = oi;
        _limelight = limelight;
        _poseTracker = poseTracker;
        requires(_driveTrainSpark);
    }

    @Override
    protected void initialize() {
        super.initialize();

        _driveState = DriveState.normal;
        _targetSighted = false;

        _angleController = new PIDController(Constants.Auto.Drive.AnglePID.kP,Constants.Auto.Drive.AnglePID.kI,Constants.Auto.Drive.AnglePID.kD, _imu, new AngleListener(), 0.1);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE, Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE);
        _angleController.setAbsoluteTolerance(Constants.Auto.Drive.AnglePID.TOLERANCE);
        _angleController.setContinuous();

        _mediumSpeed = Constants.DriveTrain.MEDIUM_SPEED;
        _slowSpeed = Constants.DriveTrain.SLOW_SPEED;

        _mediumZone = Constants.DriveTrain.MEDIUM_ZONE;
        _slowZone = Constants.DriveTrain.SLOW_ZONE;
    }

    @Override
    protected void execute() {
        super.execute();
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        _targetSighted = _limelight.isTargetSighted();
        if (!_oi.isAutoTargetPressed()) {
            _stickyLimit = 1.0;
            _lockout = false;
            if (_driveState!=DriveState.normal) {
                // Stop tracking
                _limelight.disableLEDs();
                _driveState = DriveState.normal;
            }

            if (wheelRotation == 0 && stickSpeed != 0) {
                if (!_angleController.isEnabled()) {
                    _anglePIDOut = 0;
                    double yaw = _imu.getYaw();
                    _angleController.setSetpoint(yaw);
                    _angleController.setPID(Constants.Auto.Drive.AnglePID.kP, Constants.Auto.Drive.AnglePID.kI, Constants.Auto.Drive.AnglePID.kD);
                    _angleController.enable();
                }
            } else if (_angleController.isEnabled()) {
                _angleController.disable();
                _anglePIDOut = 0;
            }
        } else {
            switch (_driveState) {
                case normal:
                    _limelight.setPipeline(Limelight.Pipeline.TapeTrackingClosest);
                    _limelight.enableLEDs();
                    _driveState = DriveState.seeking;
                    _seekMax = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                    break;
                case seeking:
                    if (_targetSighted) {
                        _turnSpeed = getTurnSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                        _driveState = DriveState.locking;
                    }
                    break;
                case locking:
                    if (System.currentTimeMillis() > _lockEnd || (_limelight.isTargetSighted() && _limelight.isTargetCentered())) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        _limelight.setPipeline(Limelight.Pipeline.TapeTrackingClosest);
                        metric("Pipeline", Limelight.Pipeline.TapeTrackingClosest.name());
                        _driveState = DriveState.tracking;
                    }
                    _turnSpeed = getTurnSpeed();
                    break;
                case tracking:
                    _turnSpeed = getTurnSpeed();
                    break;
            }
        }




        metric("State", _driveState.name());
        stickSpeed = limitSpeed(stickSpeed);
        if(!_oi.isOverridePressed()) {
            _driveTrainSpark.cheesyDrive(Math.min(stickSpeed, 0), 0, false, false);
        } else if (_driveState == DriveState.normal) {
            if (wheelRotation==0 && _angleController.isEnabled()) {
                metric("PID/AngleOut", _anglePIDOut);
                metric("PID/Yaw", _imu.getYaw());
                _driveTrainSpark.cheesyDrive(stickSpeed, stickSpeed==0 ?  0 :_anglePIDOut, false, true);
            } else {
                _driveTrainSpark.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed(), false);
            }
        } else {
            _driveTrainSpark.cheesyDrive(stickSpeed, _turnSpeed, false, true);
        }
        metric("StickSpeed", stickSpeed);
        metric("StickRotation", wheelRotation);
        metric("LeftPower", _driveTrainSpark.getLeftPower());
        metric("RightPower", _driveTrainSpark.getRightPower());
        metric("TurnSpeed", _turnSpeed);
    }

    protected double getTurnSpeed() {
        if (_lockout || !_limelight.isTargetSighted()) { return 0; }
        double distance = _limelight.getTargetDistance();

        _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;

        if (distance > 0 &&  distance < Constants.Auto.Drive.MIN_TRACK_DISTANCE) {
            garbageCount++;
            if (garbageCount > Constants.Auto.Drive.MAX_GARBAGE) {
                _lockout = true;
            }
            return 0;
        }
        garbageCount = 0;
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yaw = _imu.getYaw();

        long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
        BasicPose pose = (BasicPose)_poseTracker.get(timeKey);

        double poseAngle = pose == null ? yaw : pose.getAngle();

        double offsetCompenstaion = yaw - poseAngle;
        double targetAngle = limeLightAngle - offsetCompenstaion;

        return targetAngle * STEER_K;
    }

    private double limitSpeed(double speed) {
        double limit = 1;
        if (_driveState!=DriveState.normal) {
            if(_limelight.isTargetSighted()) {
                _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;
                double distance = _limelight.getTargetDistance();
                metric("TargetDistance", distance);
                if (distance  > 0) {
                    if (distance < _mediumZone) {
                        limit = _mediumSpeed;
                        _stickyLimit = limit;
                    }
                    if (distance < _slowZone) {
                        limit = _slowSpeed;
                        _stickyLimit = limit;
                    }
                }
            } else if (System.currentTimeMillis() > _seekMax){
                metric("TargetDistance", -999);
                // We've been seeking for more than the max allowed...slow the robot down!
                _oi.pulseDriver(1);
            }
        }
        limit = Math.min(limit, _stickyLimit);
        double limited = Helpers.limit(speed, -limit, limit);
        metric("limit", limit);
        metric("limited", limited);
        return limited;
    }



    @Override
    protected boolean isFinished() {
        return false;
    }
    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }

    }

    public enum DriveState {
        normal(0),
        seeking(1),
        locking(2),
        tracking(3),
        lost(4);

        private int _value;

        DriveState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }


}
