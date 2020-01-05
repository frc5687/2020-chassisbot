package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.Turret;
import org.frc5687.deepspace.chassisbot.utils.Limelight;
import org.frc5687.deepspace.chassisbot.utils.PoseTracker;

import static org.frc5687.deepspace.chassisbot.Constants.Turret.*;

public class DriveTurret extends OutliersCommand {

    private OI _oi;
    private Turret _turret;
    private PoseTracker _poseTracker;
    private Limelight _limelight;

    private TurretState _turretState = TurretState.normal;

    private boolean _targetSighted;

    private long _lockEnd;

    private double _prevTargetAngle;
    private double _targetAngle;
    private double _rotationSpeed;
    private double _rotationK;
    private double _turretAngle;


    public DriveTurret(Turret turret, OI oi, Limelight limelight, PoseTracker poseTracker) {
        _turret = turret;
        _oi = oi;
        _limelight = limelight;
        _poseTracker = poseTracker;
        requires(_turret);
    }

    @Override
    protected void initialize() {
        super.initialize();
        _turretState = TurretState.normal;
        _targetSighted = false;
    }

    @Override
    protected void execute()
    {
        super.execute();

        double turretRotation = _oi.getTurretRotation();

        _targetSighted = _limelight.isTargetSighted();

        if (_turret.isHallTriggered()) {
            if (_turret.getTurrentAngle() > MID_TURRET_ANGLE && turretRotation > 0) {
                turretRotation = 0;
            } else if (_turret.getTurrentAngle() < MID_TURRET_ANGLE && turretRotation < 0) {
                turretRotation = 0;
            }
        }
        if (!_oi.isAutoTargetPressed()) {
            if (_turretState != TurretState.normal) {
                // Stop tracking
                _limelight.disableLEDs();
                _turretState = TurretState.normal;
            }
        } else {
            switch (_turretState) {
                case normal:
                    // Start seeking
                    _limelight.enableLEDs();
                    _turretState = TurretState.seeking;
                    break;
                case seeking:
                    if (_targetSighted) {
                        _rotationSpeed = getRotationSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.Turret.LOCK_TIME;
                        _turretState = TurretState.locking;
                    }
                    break;
                case seekingport:
                    if (_targetSighted) {
                        _rotationSpeed = getRotationSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.Turret.LOCK_TIME;
                        _turretState = TurretState.lockingport;
                    }
                    break;
                case locking:
                    if (System.currentTimeMillis() > _lockEnd || (_limelight.isTargetSighted() && _limelight.isTargetCentered())) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        _turretState = TurretState.tracking;
                    }
                    _rotationSpeed = getRotationSpeed();
                    break;
                case lockingport:
                    if (System.currentTimeMillis() > _lockEnd || (_limelight.isTargetSighted() && _limelight.isTargetCentered())) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        //_limelight.setPipeline(Limelight.Pipeline.TapeTrackingClosest);
                        //metric("Pipeline", Limelight.Pipeline.TapeTrackingClosest.name());
                        _turretState = TurretState.tracking;
                    }
                    _rotationSpeed = getRotationSpeed();
                    break;
                case tracking:
                    _rotationSpeed = getRotationSpeed();
                    break;
            }
        }

        if (_turretState == TurretState.normal) {
            _turret.setTurretSpeed(turretRotation);
        } else {
            _turret.setTurretSpeed(_rotationSpeed);
        }
    }
    protected double getRotationSpeed() {
        if (!_limelight.isTargetSighted()) { return 0; }

        if (_turret.isHallTriggered()) {
            _turret.resetTurretEncoder();
        }
        _prevTargetAngle =_targetAngle;
        double limeLightAngle = _limelight.getHorizontalAngle();
        _turretAngle = _turret.getTurrentAngle();


        // Find the pose of the robot _when the picture was taken_
//        long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
//        TurretPose pose = (TurretPose)_poseTracker.get(timeKey);

        // Get the angle from the pose if one was found--otherwise use turretAngle
//        double poseAngle = pose == null ? _turretAngle : pose.getAngle();

        // Now adjust the limelight angle based on the change in turretAngle from when the picture was taken to now
//        double offsetCompensation = _turretAngle - poseAngle;
        _targetAngle = limeLightAngle ;
//        _targetAngle = limeLightAngle - offsetCompensation;
//        if (_targetAngle > MAX_TURRET_ANGLE) {
//            _targetAngle = _targetAngle - MAX_TURRET_ANGLE;
//        } else if (_targetAngle < MIN_TURRET_ANGLE) {
//            _targetAngle = _targetAngle + MAX_TURRET_ANGLE;
//        }

        _rotationK = ROTATION_K;

//        metric("Pose", pose==null?0:pose.getMillis());
        metric("turretAngle", _turretAngle);
//        metric("PoseAngle", poseAngle);
        metric("LimelightAngle", limeLightAngle);
        metric("targetAngle", _targetAngle);
        metric("Previous Target Angle", _prevTargetAngle);
        metric("Rotation K", _rotationK);

        return _targetAngle * _rotationK;
    }



    @Override
    protected boolean isFinished() {
        return false;
    }

    public enum TurretState {
        normal(0),
        seeking(1),
        seekingport(2),
        locking(3),
        lockingport(4),
        tracking(5),
        lost(6);

        private int _value;

        TurretState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
