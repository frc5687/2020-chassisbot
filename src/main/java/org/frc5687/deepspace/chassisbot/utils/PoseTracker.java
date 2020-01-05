package org.frc5687.deepspace.chassisbot.utils;

import edu.wpi.first.wpilibj.Notifier;

public class PoseTracker extends OutliersProxy {

    private static final double DEFAULT_PERIOD = 0.05; // Seconds
    private static final int DEFAULT_HISTORY = 1; // Seconds
    private static final long EPOCH = 1552693300000L;
    int _nextwrite = 0;
    private boolean _async;
    private double _period;
    private int _history;
    private int _bufferSize;
    private IPoseTrackable _trackable;
    private Pose[] _poses;

    public PoseTracker(IPoseTrackable trackable, boolean async, double period, int history) {
        _trackable = trackable;
        _async = async;
        _period = period;
        _history = history;
        _bufferSize = (int)(_history * _period * 1000);
        _poses = new Pose[_bufferSize];

        if (async) {
            new Notifier(this::collect).startPeriodic(_period);
        }
    }

    public PoseTracker(IPoseTrackable trackable, double period, int history) {
        this(trackable, true, period, history);
    }

    public PoseTracker(IPoseTrackable trackable) {
        this(trackable, true, DEFAULT_PERIOD, DEFAULT_HISTORY);
    }

    public PoseTracker(double period, int history) {
        this(null, false, period, history);
    }

    public PoseTracker() {
        this(null, false, DEFAULT_PERIOD, DEFAULT_HISTORY);
    }

    synchronized public void reset() {
        _nextwrite = 0;
        _poses = new Pose[_bufferSize];
    }

    public void collect(Pose pose) {
        if (_async) {
            throw new RuntimeException("Client code must not call collect on an asynchronous PoseTracker.");
        }
        add(pose);
    }

    synchronized private void collect() {
        if (_async && _trackable == null) {
            throw new RuntimeException("Asynchronous collect called on a PoseTracker instance with no trackable.");
        }
        Pose pose = _trackable.getPose();
        metric("PoseCollected", pose.getMillis() - EPOCH);
        add(pose);
    }


    synchronized public void add(Pose pose) {
        _poses[_nextwrite] = pose;
        _nextwrite++;
        if (_nextwrite >= _bufferSize) {
            _nextwrite = 0;
        }
    }

    synchronized public Pose get(long millis) {
        Pose afterPose = null;
        int read = _nextwrite - 1;
        while (true) {
            if (read < 0) {
                read = _bufferSize - 1;
            }
            // If there's nothing there, we're out of buffer
            if (_poses[read] == null) {
                metric("PoseFound", afterPose==null?0:afterPose.getMillis() - EPOCH );
                return afterPose;
            }

            // If the pose time is before the requested time, we'er out of buffer
            if (_poses[read].getMillis() < millis) {
                metric("PoseFound", afterPose==null?0:afterPose.getMillis() - EPOCH );
                return afterPose;
            }

            // Otherwise grab the pose
            afterPose = _poses[read];

            read--;
        }

    }

    synchronized public Pose getLatestPose() {
        Pose afterPose = null;
        int read = _nextwrite - 1;
        if (read < 0) {
            read = _bufferSize - 1;
        }
        // If there's nothing there, we're out of buffer
        if (_poses[read] == null) {
            return null;
        } else {
            return _poses[read];
        }
    }


    synchronized public void updateDashboard() {
        Pose pose = getLatestPose();
        if (pose!=null) {
            pose.updateDashboard("PoseTracker");
        }
    }


}
