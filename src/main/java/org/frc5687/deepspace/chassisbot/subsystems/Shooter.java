package org.frc5687.deepspace.chassisbot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;
import org.frc5687.deepspace.chassisbot.commands.Shoot;

import static org.frc5687.deepspace.chassisbot.Constants.Shooter.*;

public class Shooter extends OutliersSubsystem {

    private Robot _robot;
    private CANSparkMax _rightSpark;
    private CANSparkMax _leftSpark;
    private CANEncoder _boreEncoder;



    public Shooter(Robot robot) {
        _robot = robot;
        try {
            _rightSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_SHOOTER_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
            _leftSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_SHOOTER_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightSpark.setInverted(RIGHT_INVERTED);
            _leftSpark.setInverted(LEFT_INVERTED);
            _boreEncoder = _rightSpark.getAlternateEncoder(AlternateEncoderType.kQuadrature, COUNTS_PER_REV);
        } catch (Exception e) {
            error("Unable to allocate shooter controllers" + e.getMessage());
        }
    }

    @Override
    public void periodic() {
        setDefaultCommand(new Shoot(this, _robot.getOI()));
    }

    @Override
    public void updateDashboard() {
        metric("Position", getPosition());
        metric("Velocity", getVelocity());
        metric("LeftSpeed", getLeftSpeed());
        metric("RightSpeed", getRightSpeed());
    }

    public double getLeftSpeed() {
        return _leftSpark.get();
    }
    public double getRightSpeed() { return _rightSpark.get(); }

    public void setSpeed(double speed) {
//        _leftSpark.set(speed);
        _rightSpark.set(speed);
    }
    //Native units are rotations.
    public double getPosition() {
        return _boreEncoder.getPosition();
    }
    //Native units are RPM.
    public double getVelocity() {
        return _boreEncoder.getVelocity();
    }

    public void setCoastMode() {
        _leftSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }


}
