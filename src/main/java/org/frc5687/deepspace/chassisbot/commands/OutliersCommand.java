package org.frc5687.deepspace.chassisbot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.frc5687.deepspace.chassisbot.utils.ILoggingSource;
import org.frc5687.deepspace.chassisbot.utils.MetricTracker;
import org.frc5687.deepspace.chassisbot.utils.RioLogger;

public abstract class OutliersCommand extends CommandBase implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersCommand() {
    }

    public OutliersCommand(double timeout) {
        super.withTimeout(timeout);
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker!=null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker!=null) {
            _metricTracker.put(name, value);
        }
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        if (_metricTracker!=null) {
            _metricTracker.put(name, value);
        }
    }

    protected void logMetrics(String... metrics) {
        _metricTracker = MetricTracker.createMetricTracker(getClass().getSimpleName(), metrics);
        _metricTracker.pause();
    }

    @Override
    public void initialize() {
        super.initialize();
        if (_metricTracker!=null) {
            _metricTracker.resume();
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (_metricTracker!=null) {
            _metricTracker.pause();
        }
    }

}

