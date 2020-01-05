package org.frc5687.deepspace.chassisbot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.chassisbot.utils.ILoggingSource;
import org.frc5687.deepspace.chassisbot.utils.MetricTracker;
import org.frc5687.deepspace.chassisbot.utils.RioLogger;
import org.frc5687.deepspace.chassisbot.utils.Version;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/***
 * We use this base class for our logging, metric tracking, and identity tracking framework.
 * The main Robot class should extend it and each override needs to start with a super call.
 */
public abstract class OutliersRobot extends TimedRobot implements ILoggingSource {
    private int _updateTick = 0;
    protected String _name;
    public static IdentityMode _identityMode = IdentityMode.competition;
    protected RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    protected RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    @Override
    public void robotInit() {
        super.robotInit();
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        metric("Branch", Version.BRANCH);
        info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");

        // Periodically flushes metrics (might be good to configure enable/disable via USB config file)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        MetricTracker.flushAll();
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            updateDashboard();
        }
    }

    abstract protected void updateDashboard();

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        ourPeriodic();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        ourPeriodic();
    }

    protected void ourPeriodic() {
        MetricTracker.newMetricRowAll();
    }

    /***
     * Helper function to read the robot identity and logging settings from the USB thumbdrive.
     * Note that if this fails (eg, there's no thumbdrive) we assume competition mode and disable logging.
     * The config file is simple.  Each line is either a comment (starting with #) or a key=value pair.
     * Valid keys are:
     * name - The name of this robot
     * mode - Whether it's the competition bot or practice bot
     * fileloglevel - Which log messages should be written to the thumbdrive.
     * dsloglevel - Which log messages should be written to the driverstation.
     */
    private void loadConfigFromUSB() {
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine())!=null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            _identityMode = IdentityMode.competition;
        }
    }

    /***
     * Helper function to process a line from the config file.
     * @param line
     */
    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) { return; }
            String[] a = line.split("=");
            if (a.length==2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        metric("name", _name);
                        break;
                    case "mode":
                        _identityMode = IdentityMode.valueOf(value.toLowerCase());
                        metric("mode", _identityMode.toString());
                        break;
                    case "fileloglevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("fileLogLevel", _fileLogLevel.toString());
                        break;
                    case "dsloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("dsLogLevel", _dsLogLevel.toString());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }


    /***
     * We use the IdentityMode enum to track which of our robots the code is running on.
     * This is helpful because many constants have to be different on the practice bot vs. the competition bot.
     * We also have a programming rio on a board with no motor controllers, so we need to account for that.
     */
    public enum IdentityMode {
        competition(0),
        practice(1),
        programming(2);

        private int _value;

        IdentityMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public IdentityMode getIdentityMode() {
        return _identityMode;
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
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

}