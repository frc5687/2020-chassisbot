package org.frc5687.deepspace.chassisbot.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;
import org.frc5687.deepspace.chassisbot.commands.DriveTurret;
import org.frc5687.deepspace.chassisbot.utils.HallEffect;
import org.frc5687.deepspace.chassisbot.utils.Limelight;
import org.frc5687.deepspace.chassisbot.utils.PoseTracker;

import static org.frc5687.deepspace.chassisbot.Constants.Turret.*;
import static org.frc5687.deepspace.chassisbot.utils.Helpers.limit;

public class Turret extends OutliersSubsystem {

    private OI _oi;
    private PoseTracker _poseTracker;
    private Limelight _limelight;
    private Robot _robot;

    private CANSparkMax _turretSpark;
    private CANEncoder _turretEncoder;
    private CANEncoder _tiltEncoder;
    private CANSparkMax _turretTiltSpark;

    private HallEffect _turretHall;

    private HallEffect _tiltHall; //turret tilts hall effect

    private double _angle0 = 0;
    private double _angle1 = 0; //tilt angle

    public Turret(Robot robot)
    {
        _robot = robot;
        _oi = robot.getOI();
        _limelight = robot.getLimelight();
        _poseTracker = robot.getPoseTracker();


        _turretSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.TURRET_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _turretTiltSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.TILT_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _turretSpark.setInverted(Constants.Turret.TURRET_MOTOR_INVERTED);
        _turretEncoder = _turretSpark.getEncoder();
        _tiltEncoder = _turretTiltSpark.getEncoder();
        _turretHall = new HallEffect(RobotMap.DIO.TURRET_HALL);
        _tiltHall = new HallEffect(RobotMap.DIO.TURRET_TILT_HALL);
    }
    @Override
    public void updateDashboard()
    {
        metric("Turret Angle", getTurrentAngle());
        metric("Turret Position", getTurretPosition());
        metric("Turret Power", getTurretPower());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveTurret(this, _oi, _limelight, _poseTracker));
    }

    public void enableBreakMode()
    {
        if (_turretSpark == null) { return; }
        _turretSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableBreakModeTilt()  //enables breaking for protoBot's turret tilt
    {
        if(_turretTiltSpark == null) {return;}
        _turretTiltSpark.setIdleMode((CANSparkMax.IdleMode.kBrake));
    }

    public void disableBreakMode()
    {
        if (_turretSpark == null) { return; }
        _turretSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void disableBreakModeTilt()   //disables breaking for protoBot's turret tilt
    {
        if(_turretTiltSpark == null) {return;}
        _turretTiltSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setTurretSpeed(double speed)
    {
        if (_turretSpark == null) { return; }

        speed = limit(speed, MAX_REVERSE_SPEED, MAX_FORWARD_SPEED);
        _turretSpark.set(speed);
    }

    public void setTiltSpeed()
    {
        _turretTiltSpark.set(0.7);
    }

    public void getTurretTiltSpeed(double tspeed) //gets tilt's speed
    {
        if(_turretTiltSpark == null) {return;}

        tspeed = limit(tspeed, MAX_REVERSE_SPEED, MAX_REVERSE_SPEED);
        _turretTiltSpark.set(tspeed); //sets tilt speed to tspeed
    }

    public  void setTurretTiltSpeed(double tiltSpeed)  //controls turret tilt speed
    {
        if(_turretTiltSpark == null) {return;}
        tiltSpeed = limit(tiltSpeed, MAX_REVERSE_SPEED, MAX_FORWARD_SPEED);
        _turretTiltSpark.set(tiltSpeed);
    }

    public boolean isHallTriggered()
    {
        return _turretHall.get();
    }

    public boolean isHallTriggeredTilt() //gets tilt value for turret
    {
        return _tiltHall.get();
    }

    public double getTurretPosition()
    {
        return _turretEncoder.getPosition();
    }

    public  double getTurretTiltPosition(){return _tiltEncoder.getPosition();} //receives turrets tilt position

    public double getTurretPower() {return _turretSpark.get(); }

    public double getTurretTiltPower() {return _turretTiltSpark.get();} //gets turret tilt power

    public void resetTurretEncoder()
    {
        _angle0 = getTurrentAngle() > MID_TURRET_ANGLE ? MAX_TURRET_ANGLE : MIN_TURRET_ANGLE;
        _turretEncoder.setPosition(0);
    }

    public  void resetTurretTiltEncoder() //resets tilt encoder
    {
        _angle1 = getTurrentAngle() > MID_TURRET_ANGLE ? MAX_TURRET_ANGLE : MIN_TURRET_ANGLE;
        _tiltEncoder.setPosition(0);
    }

    public double getTurrentAngle() //It's spelled "Turret" not "Turrent"
    {
        return _angle0 + (getTurretPosition() / TICKS_PER_DEGREES);
    }

    public  double getTurretTiltAngle() //gets turrets tilt angle in degrees
    {
        return  _angle1 + (getTurretPosition() / TICKS_PER_DEGREES);
    }
}