package org.frc5687.deepspace.chassisbot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.Shooter;

public class ShootSpeedSetpoint extends OutliersCommand {

    private Shooter _shooter;
    private OI _oi;
    private double _speed;

    public ShootSpeedSetpoint(Shooter shooter, OI oi, double speed) {
        _shooter = shooter;
        _oi = oi;
        _speed = speed;
        addRequirements(_shooter);
        logMetrics("Velocity");
    }
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/ShootSpeedSetpoint", true);
        super.initialize();
        _shooter.setCoastMode();
    }

    @Override
    public void execute() {
         _shooter.setSpeed(_speed);
         metric("Velocity", _shooter.getVelocity());
    }

    @Override
    public boolean isFinished() {
        return _oi.isKillAllPressed();
    }

}
