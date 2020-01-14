package org.frc5687.deepspace.chassisbot.commands;

import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.Intake;

public class IntakeSpin extends OutliersCommand {
    private OI _oi;
    private Intake _intake;
    private double _rotationSpeed;

    public IntakeSpin(Intake intake, OI oi) {
        _intake = intake;
        _oi = oi;
        requires(_intake);
    }

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void execute() {
        double intakeSpeed = _oi.getIntakeSpeed();
        _intake.setSpeed(intakeSpeed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
