package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.Robot;

public class KillAll extends OutliersCommand {
    private boolean _finished;
    private Robot _robot;

    public KillAll(Robot robot) {
        requires(robot.getSparkMaxDriveTrain());
        _robot = robot;
    }

    @Override
    protected void initialize() {
        _robot.getSparkMaxDriveTrain().enableBrakeMode();

        _robot.getLimelight().disableLEDs();
        _finished = true;
        error("Initialize KillAll Command");
    }

    @Override
    protected void end() {
        error("Ending KillAll Command");
    }

    @Override
    protected boolean isFinished() {
        return _finished;
    }
}
