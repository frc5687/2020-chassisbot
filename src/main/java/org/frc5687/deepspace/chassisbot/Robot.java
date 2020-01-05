package org.frc5687.deepspace.chassisbot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import org.frc5687.deepspace.chassisbot.commands.KillAll;
import org.frc5687.deepspace.chassisbot.subsystems.*;
import org.frc5687.deepspace.chassisbot.utils.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements ILoggingSource, IPoseTrackable {

    public static IdentityMode identityMode = IdentityMode.competition;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    private int _updateTick = 0;

    private String _name;
    private OI _oi;

    private AHRS _imu;
    private Limelight _limelight;
    //private VictorSPDriveTrain _driveTrainVictor;
    private SparkMaxDriveTrain _driveTrainSpark;
    private Shifter _shifter;

    private PDP _pdp;
    private PoseTracker _poseTracker;

    private boolean _fmsConnected;

    private Command _autoCommand;

    private Trajectory _leftSideLeftTrajectory;
    private Trajectory _leftSideRightTrajectory;
    private Trajectory _rightSideLeftTrajectory;
    private Trajectory _rightSideRightTrajectory;

    private UsbCamera _driverCamera;

    private AutoChooser _autoChooser;

    /**
     * This function is setRollerSpeed when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        metric("Branch", Version.BRANCH);
        info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        info("Robot " + _name + " running in " + identityMode.toString() + " mode");

        // Periodically flushes metrics (might be good to configure enable/disable via USB config file)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);

        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _pdp = new PDP();
        _limelight = new Limelight("limelight");
        _autoChooser = new AutoChooser(true);


        // Then subsystems....
        //_driveTrainVictor = new VictorSPDriveTrain(this);
        _shifter = new Shifter(this);
        _driveTrainSpark = new SparkMaxDriveTrain(this);

        _poseTracker = new PoseTracker(this);



        // Must initialize buttons AFTER subsystems are allocated...
        _oi.initializeButtons(this);

        // Initialize the other stuff
        _limelight.disableLEDs();
        _limelight.setStreamingMode(Limelight.StreamMode.PIP_SECONDARY);

        try {
            _driverCamera = CameraServer.getInstance().startAutomaticCapture(0);
            _driverCamera.setResolution(160, 120);
            _driverCamera.setFPS(30);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
        }

        try {
            initializeTrajectories();
        } catch (IOException ioe) {
            DriverStation.reportError(ioe.getMessage(), true);
        }

    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        updateDashboard();
        _oi.poll();
        update();
    }

    private void initializeTrajectories() throws IOException {
        var  path = "LeftFarRocket";
        info("Loading trajectories for " + path);
        _leftSideLeftTrajectory = PathfinderFRC.getTrajectory(path + ".right");
        _leftSideRightTrajectory = PathfinderFRC.getTrajectory(path + ".left");

        path = "RightFarRocket";
        info("Loading trajectories for " + path);
        _rightSideLeftTrajectory = PathfinderFRC.getTrajectory(path + ".right");
        _rightSideRightTrajectory = PathfinderFRC.getTrajectory(path + ".left");
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        _fmsConnected =  DriverStation.getInstance().isFMSAttached();
        _driveTrainSpark.enableBrakeMode();
        _limelight.disableLEDs();
        _limelight.setStreamingMode(Limelight.StreamMode.PIP_SECONDARY);
        _autoCommand = null;

        switch (_autoChooser.getSelectedMode()) {
            case LeftDoubleRocket:
                //left side
//                _autoCommand = new TwoHatchCloseAndFarRocket(this, false, true, _leftSideLeftTrajectory, _leftSideRightTrajectory);
                break;
            case RightDoubleRocket:
                //right side
//                _autoCommand = new TwoHatchCloseAndFarRocket(this, false, false, _rightSideLeftTrajectory, _rightSideRightTrajectory);
                break;
        }
        if (_autoCommand!=null) {
            _autoCommand.start();
        }
    }

    public void teleopInit() {
        _fmsConnected =  DriverStation.getInstance().isFMSAttached();
        //_limelight.disableLEDs();
    }

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

    private void ourPeriodic() {
        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();

        if (_oi.isKillAllPressed()) {
            new KillAll(this).start();
        }

        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        //_limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        MetricTracker.flushAll();
    }


    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _autoChooser.updateDashboard();
            _updateTick = 0;
            _oi.updateDashboard();
            //_driveTrainVictor.updateDashboard();
            _shifter.updateDashboard();
            _driveTrainSpark.updateDashboard();
            _pdp.updateDashboard();
            _limelight.updateDashboard();
        }
    }


    private void loadConfigFromUSB() {    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
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
            identityMode = IdentityMode.competition;
        }
    }

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
                        identityMode = IdentityMode.valueOf(value.toLowerCase());
                        metric("mode", identityMode.toString());
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

    private boolean _wasShocked = false;

    private void update() {

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

    public OI getOI() {
        return _oi;
    }
    public AHRS getIMU() { return _imu; }
    //public VictorSPDriveTrain getVictorSPDriveTrain() { return _driveTrainVictor; }
    public SparkMaxDriveTrain getSparkMaxDriveTrain() { return _driveTrainSpark; }
    public Shifter getShifter() { return _shifter; }

    public PDP getPDP() { return _pdp; }
    public Limelight getLimelight() { return _limelight; }
    public PoseTracker getPoseTracker() { return _poseTracker; }

    @Override
    public Pose getPose() {
        return new BasicPose(_imu.getYaw(), _driveTrainSpark.getLeftDistance(), _driveTrainSpark.getRightDistance(), _driveTrainSpark.getDistance());
    }

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
        return identityMode;
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
}