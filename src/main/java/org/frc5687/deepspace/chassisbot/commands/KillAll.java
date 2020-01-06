package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.Robot;

public class KillAll extends OutliersCommand {
    private boolean _finished;
    private Robot _robot;

    public KillAll(Robot robot) {
        addRequirements(robot.getDriveTrain());
        _robot = robot;
    }

    @Override
    public void initialize() {
        _robot.getDriveTrain().enableBrakeMode();

        _robot.getLimelight().disableLEDs();
        _finished = true;
        error("Initialize KillAll Command");
    }

    @Override
    public void end(boolean interrupted)  {
        error("Ending KillAll Command");
    }

    @Override
    public boolean isFinished() {
        return _finished;
    }
}
