package org.frc5687.deepspace.chassisbot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;

public class Intake extends OutliersSubsystem {
    private Robot _robot;

    private CANSparkMax _intakeSpark;
    private DoubleSolenoid _intakeSolenoid;
    private OI _oi;

    public Intake (Robot robot) {
        _robot = robot;
        _intakeSolenoid = new DoubleSolenoid(RobotMap.PCM.SHIFTER_HIGH, RobotMap.PCM.SHIFTER_LOW);//check if shifter high and shifter low should be changed
        _oi = robot.getOI();

        _intakeSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.INTAKE_NEO, CANSparkMaxLowLevel.MotorType.kBrushless);
        _intakeSpark.setInverted(Constants.Turret.TURRET_MOTOR_INVERTED);

    }

    public Intake.Position getPosition() {
        DoubleSolenoid.Value current = _intakeSolenoid.get();
        if (current== Intake.Position.HIGH.getSolenoidValue()) {
            return Intake.Position.HIGH;
        } else if (current== Intake.Position.LOW.getSolenoidValue()) {
            return Intake.Position.LOW;
        }
        return Intake.Position.UNKNOWN;
    }

    @Override
    public void updateDashboard()
    {
        metric("INTAKE POWER", getIntakePower());
        metric("INTAKE POSITION", getPosition()== Intake.Position.HIGH ? "Intake from Human Player" : (getPosition() == Intake.Position.LOW ? "Intake from Ground" : "Unknown"));
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(null);
    }

    public void raiseIntake() { _intakeSolenoid.set(DoubleSolenoid.Value.kReverse); }

    public void lowerIntake() { _intakeSolenoid.set(DoubleSolenoid.Value.kForward); }

    /*public void updateDashboard() {
        metric("Intake Position", getPosition()== Intake.Position.HIGH ? "Intake from Human Player" : (getPosition() == Intake.Position.LOW ? "Intake from Ground" : "Unknown"));
    }*/

    public enum Position {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        HIGH(DoubleSolenoid.Value.kReverse),
        LOW(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        Position(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }

    }

    public void setSpeed(double speed) {
        _intakeSpark.set(speed);
    }

    public double getIntakePower() {return _intakeSpark.get(); }

}