package org.frc5687.deepspace.chassisbot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.Robot;
import org.frc5687.deepspace.chassisbot.RobotMap;

public class Shifter extends OutliersSubsystem {
    private Robot _robot;

    private DoubleSolenoid shifterSolenoid;
    private Compressor compressor;
    private long waitPeriodEndTime = 0;
    private boolean autShiftEnabled = false;

    public Shifter(Robot robot) {
        _robot = robot;
        shifterSolenoid = new DoubleSolenoid(RobotMap.PCM.SHIFTER_HIGH, RobotMap.PCM.SHIFTER_LOW);
    }

    @Override
    protected void initDefaultCommand() {
    }

    public void shift(Gear gear, boolean auto) {
        shifterSolenoid.set(gear.getSolenoidValue());
        waitPeriodEndTime = System.currentTimeMillis() + (auto ? Constants.Shifter.AUTO_WAIT_PERIOD : Constants.Shifter.MANUAL_WAIT_PERIOD);
//        if (gear==Gear.HIGH) {
//            if (!DriverStation.getInstance().isAutonomous()) {
//                _robot.getOI().rumbleRight();
//            }
//        }
//        if (gear==Gear.LOW) {
//            if (!DriverStation.getInstance().isAutonomous()) {
//                _robot.getOI().rumbleLeft();
//            }
//        }
//        _robot.getLEDStrip().setGear(gear);
    }

    public boolean waitPeriodElapsed() {
        return System.currentTimeMillis() > waitPeriodEndTime;
    }

    public Gear getGear() {
        DoubleSolenoid.Value current = shifterSolenoid.get();
        if (current==Gear.HIGH.getSolenoidValue()) {
            return Gear.HIGH;
        } else if (current== Gear.LOW.getSolenoidValue()) {
            return Gear.LOW;
        }
        return Gear.UNKNOWN;
    }

    public boolean isCompressorAtPressure() {
        return compressor.getPressureSwitchValue();
    }

    public boolean isCommpressorEnabled() {
        return compressor.enabled();
    }

    public double getCompressorCurrent() {
        return compressor.getCompressorCurrent();
    }

    public void updateDashboard() {
        metric("Gear", getGear()==Gear.HIGH ? "High" : (getGear() == Gear.LOW ? "Low" : "Unknown"));
    }

    public enum Gear {
        UNKNOWN(DoubleSolenoid.Value.kOff),
        HIGH(DoubleSolenoid.Value.kReverse),
        LOW(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value solenoidValue;

        Gear(DoubleSolenoid.Value solenoidValue) {
            this.solenoidValue = solenoidValue;
        }

        public DoubleSolenoid.Value getSolenoidValue() {
            return solenoidValue;
        }

    }

    public boolean isAutShiftEnabled() {
        return autShiftEnabled;
    }

    public void setAutShiftEnabled(boolean enabled) {
        autShiftEnabled = enabled;
    }

}
