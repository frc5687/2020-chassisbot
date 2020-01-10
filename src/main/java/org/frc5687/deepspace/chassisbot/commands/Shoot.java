package org.frc5687.deepspace.chassisbot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.chassisbot.OI;
import org.frc5687.deepspace.chassisbot.subsystems.Shooter;

public class Shoot extends OutliersCommand {
    private Shooter _shooter;
    private OI _oi;

    public Shoot(Shooter shooter, OI oi) {
        _shooter = shooter;
        _oi = oi;
        addRequirements(_shooter);
        logMetrics("Velocity");
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("MetricTracker/Shoot", true);
        super.initialize();
        _shooter.setCoastMode();
    }

    @Override
    public void execute() {
        double speed = _oi.getShooterSpeed();
        double requestedSpeed = _shooter.getRightSpeed();

        if (speed < requestedSpeed) {
            speed += 0.02;
        } else {
            _shooter.setSpeed(speed);
        }
//        } else if (speed > requestedSpeed) {
//            speed -= 0.02;
//        }


        metric("RequestedSpeed", speed);
        metric("Velocity", _shooter.getVelocity());
        metric("Position", _shooter.getPosition());
        metric("Speed", _shooter.getLeftSpeed());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
