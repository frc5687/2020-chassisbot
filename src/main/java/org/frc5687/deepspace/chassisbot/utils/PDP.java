package org.frc5687.deepspace.chassisbot.utils;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PDP extends PowerDistributionPanel {
    private double[] cache;
    private Thread _thread;
    private PDPCacheUpdater _pdpCacheUpdater;
    public PDP() {
        super();
        cache = new double[16];
        _pdpCacheUpdater = new PDPCacheUpdater(this);
        _thread = new Thread(_pdpCacheUpdater);
        _thread.start();
    }

    @Override
    public double getCurrent(int channel) {
        return getCurrent(channel, false);
    }

    public double getCurrent(int channel, boolean overrideCache) {
        if (overrideCache) {
            return super.getCurrent(channel);
        }
        return cache[channel];
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("PDP/Current/0", getCurrent(0));
        SmartDashboard.putNumber("PDP/Current/1", getCurrent(1));
        SmartDashboard.putNumber("PDP/Current/2", getCurrent(2));
        SmartDashboard.putNumber("PDP/Current/3", getCurrent(3));
        SmartDashboard.putNumber("PDP/Current/4", getCurrent(4));
        SmartDashboard.putNumber("PDP/Current/5", getCurrent(5));
        SmartDashboard.putNumber("PDP/Current/6", getCurrent(6));
        SmartDashboard.putNumber("PDP/Current/7", getCurrent(7));
        SmartDashboard.putNumber("PDP/Current/8", getCurrent(8));
        SmartDashboard.putNumber("PDP/Current/9", getCurrent(9));
        SmartDashboard.putNumber("PDP/Current/10", getCurrent(10));
        SmartDashboard.putNumber("PDP/Current/11", getCurrent(11));
        SmartDashboard.putNumber("PDP/Current/12", getCurrent(12));
        SmartDashboard.putNumber("PDP/Current/13", getCurrent(13));
        SmartDashboard.putNumber("PDP/Current/14", getCurrent(14));
        SmartDashboard.putNumber("PDP/Current/15", getCurrent(15));
    }
    
    public boolean excessiveCurrent(int channel, double threshold) {
        double current = getCurrent(channel);
        if (current >= threshold) {
            RioLogger.debug(this.getClass().getSimpleName(), "PDP Channel: " + channel + " excessive at " + current);
            return true;
        }
        return false;
    }

    private class PDPCacheUpdater implements Runnable {
        private PDP _pdp;

        public PDPCacheUpdater(PDP pdp) {
            _pdp = pdp;
        }

        public void run() {
            while (true) {
                try {
                    for (int i = 0; i < 16; i++) {
                        _pdp.cache[i] = _pdp.getCurrent(i, true);
                    }
                    Thread.sleep(250);
                } catch (Exception e) {
                    RioLogger.error(_pdp,  "PDPCacheUpdater exception: " + e.toString());
                }
            }
        }
    }

}