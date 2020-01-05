package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.subsystems.Shifter;
import org.frc5687.deepspace.chassisbot.subsystems.SparkMaxDriveTrain;

public class Shift extends OutliersCommand {
    private Shifter _shifter;
    private SparkMaxDriveTrain _driveTrain;
    private Shifter.Gear gear;

    private double initialLeftSpeed, initialRightSpeed;
    private long endTime;
    private State state = State.STOP_MOTOR;
    private boolean auto;

    public Shift(SparkMaxDriveTrain driveTrain, Shifter shifter, Shifter.Gear gear, boolean auto) {
        _driveTrain = driveTrain;
        _shifter = shifter;

        requires(_driveTrain);
        requires(_shifter);

        this.gear = gear;
        this.auto = auto;
    }

    @Override
    protected void initialize() {
        info("Shifting to " + gear);
        state =   State.STOP_MOTOR;
    }

    @Override
    protected void execute() {
        switch (state) {
            case STOP_MOTOR:
                _driveTrain.pauseMotors();
                endTime = System.currentTimeMillis() + Constants.Shifter.STOP_MOTOR_TIME;
                state = State.WAIT_FOR_MOTOR;
                break;
            case WAIT_FOR_MOTOR:
                if (System.currentTimeMillis() >= endTime) state = State.SHIFT;
                break;
            case SHIFT:
                _shifter.shift(gear, auto);
                endTime = System.currentTimeMillis() + Constants.Shifter.SHIFT_TIME;
                state = State.WAIT_FOR_SHIFT;
                break;
            case WAIT_FOR_SHIFT:
                if (System.currentTimeMillis() >= endTime) state = State.START_MOTOR;
                break;
            case START_MOTOR:
                _driveTrain.resumeMotors();
                state = State.DONE;
                break;
        }
    }

    @Override
    protected boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    protected void interrupted() {
    }

    public enum State {
        STOP_MOTOR,
        WAIT_FOR_MOTOR,
        SHIFT,
        WAIT_FOR_SHIFT,
        START_MOTOR,
        DONE;
    }
}
