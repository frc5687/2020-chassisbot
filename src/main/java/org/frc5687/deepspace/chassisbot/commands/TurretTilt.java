package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.Turret;
import org.frc5687.deepspace.chassisbot.utils.HallEffect;
import org.frc5687.deepspace.chassisbot.utils.Limelight;
import org.frc5687.deepspace.chassisbot.utils.PoseTracker;

import static org.frc5687.deepspace.chassisbot.Constants.Turret.MID_TURRET_ANGLE;

public class TurretTilt extends OutliersCommand
{


    private OI _oi;                 //function and variable definitions go here
    private Turret _turretTilt;
    private PoseTracker _poseTracker;
    private Limelight _limelight;

    private DriveTurret.TurretState _turretState = TurretTilt.TurretState.normal;

    private boolean _targetTiltSighted;
    private DriveTurret.TurretState _turretTiltState;

    private long _lockEnd;

    private double _prevTargetAngle;
    private double _targetAngle;
    private double _rotationSpeed;
    private double _rotationK;
    private double _turretAngle;
    private HallEffect _turretTiltHall;
    private double _turretTiltRotation;


    @Override
    protected void initialize()
    {
        super.initialize();
        _turretTiltState = DriveTurret.TurretState.normal;
        _targetTiltSighted = false;
    }

    @Override
    protected boolean isFinished()
    {
        return false;
    }

    public TurretTilt(Turret turret, OI oi, Limelight limelight, PoseTracker poseTracker) //some one explain to me what the purpose of this code is
    {
        _turretTilt = turret;
        _oi = oi;
        _limelight = limelight;
        _poseTracker = poseTracker;
        requires(_turretTilt);
    }

    public enum TurretState //sets states for tiltPos
    {
        tnormal(0),
        tseeking(1),
        tseekingport(2),
        tlocking(3),
        tlockingport(4),
        ttracking(5),
        tlost(6);

        public static DriveTurret.TurretState normal;
        private int _tvalue;

        TurretState(int value) {
            this._tvalue = value;
        }

        public int getValue() {
            return _tvalue;
        }
    }

    @Override
    protected void execute() //code to move motor
    {
        super.execute();
        if(_limelight.isTargetSighted()) //if target is sighted
        {
            double tiltAng = _turretTilt.getTurretTiltAngle(); //gets tilt angle
            double tiltEnco = _turretTilt.getTurretTiltPosition(); //gets tilt position //maybe not needed
            double limeLightVAng = _limelight.getVerticalAngle(); //gets limeLight vertical angle
            while(tiltAng != limeLightVAng) //While tilt angle is not equal to limeLight vertical angle
            {
                _turretTilt.setTiltSpeed(); //sets turrets DC motor to move
            }
        }
    }
}
