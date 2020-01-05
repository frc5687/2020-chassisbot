package org.frc5687.deepspace.chassisbot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.chassisbot.utils.ILoggingSource;
import org.frc5687.deepspace.chassisbot.utils.MetricTracker;
import org.frc5687.deepspace.chassisbot.utils.RioLogger;

public abstract class OutliersCommand extends Command implements ILoggingSource {
    private MetricTracker _metricTracker;

    public OutliersCommand() {
    }

    public OutliersCommand(double timeout) {
        super.setTimeout(timeout);
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
    protected void initialize() {
        super.initialize();
        if (_metricTracker!=null) {
            _metricTracker.resume();
        }
    }

    @Override
    protected void end() {
        super.end();
        if (_metricTracker!=null) {
            _metricTracker.pause();
        }
    }

    @Override
    protected void interrupted() {
        super.interrupted();
        if (_metricTracker!=null) {
            _metricTracker.pause();
        }
    }
}

