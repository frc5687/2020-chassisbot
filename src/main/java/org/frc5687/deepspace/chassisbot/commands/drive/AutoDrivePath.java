package org.frc5687.deepspace.chassisbot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.commands.OutliersCommand;
import org.frc5687.deepspace.chassisbot.subsystems.SparkMaxDriveTrain;
import org.frc5687.deepspace.chassisbot.utils.BasicPose;
import org.frc5687.deepspace.chassisbot.utils.Limelight;
import org.frc5687.deepspace.chassisbot.utils.PoseTracker;

public class AutoDrivePath extends OutliersCommand {
    private Trajectory _leftTrajectory;
    private Trajectory _rightTrajectory;
    private DistanceFollower _leftFollower;
    private DistanceFollower _rightFollower;
    private Notifier _pathNotifier;

    private SparkMaxDriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private PoseTracker _poseTracker;

    private double kPangle = .001;
    private double kIangle = .0001;
    private double kDangle = .001;
    private int _index = 0;
    private int _trackingThreshold;
    private String _path;
    private State _state;

    private boolean _useVisionAngle;
    private double _visionAngle;

    private boolean _backwards;
    private int _direction;

    public AutoDrivePath(SparkMaxDriveTrain driveTrain, AHRS imu, Limelight limelight, PoseTracker poseTracker, Trajectory leftTrajectory, Trajectory rightTrajectory, int trackingSegments, boolean backwards) {
        requires(driveTrain);
        _driveTrain = driveTrain;
        _imu = imu;
        _limelight = limelight;
        _poseTracker = poseTracker;

        _backwards = backwards;
        _direction = backwards ? -1 : 1;

        _leftTrajectory = leftTrajectory;
        _rightTrajectory = rightTrajectory;
        info("Left has " + _leftTrajectory.length() + " segments.");
        info("Right has " + _rightTrajectory.length() + " segments.");
        _trackingThreshold = _leftTrajectory.length() - trackingSegments;
        // logMetrics("Segment", "State", "LeftDistance", "RightDistance", "LeftSpeed","RightSpeed","Yaw","Heading","VisionHeading","DesiredHeading","HeadingDifference", "Turn","LeftOutput","RightOutput");

    }

    @Override
    protected void initialize() {
        SmartDashboard.putBoolean("MetricTracker/AutoDrivePath", true);
        super.initialize();
        _driveTrain.resetDriveEncoders();
        info("Allocating followers");

        _leftFollower = new DistanceFollower(_backwards ? _rightTrajectory : _leftTrajectory);
        _rightFollower = new DistanceFollower(_backwards ? _leftTrajectory : _rightTrajectory);

        _state = State.normal;

        error("COnfiguring follower PID");
        _leftFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);
        _rightFollower.configurePIDVA(0.1, 0.0, 0.001, 1 / Constants.DriveTrain.MAX_SPEED_IPS, 0);

        _pathNotifier = new Notifier(this::run);
        _pathNotifier.startPeriodic(_leftTrajectory.get(0).dt);
        _index = 0;
    }

    @Override
    protected void execute() {
    }
    public void run() {
        _index++;
        metric("Segment", _index);
        metric("State",_state.name());
        switch (_state) {
            case normal:
                if (_index >= _trackingThreshold) {
                    // Start looking for the target!
                    _limelight.enableLEDs();
                    _limelight.setPipeline(0);
                    _state = State.tracking;
                }
                break;
            case tracking:
                if (_limelight.isTargetSighted()) {
                    if (_limelight.getTargetDistance() > Constants.Auto.Drive.MIN_TRACK_DISTANCE) {
                        double limeLightAngle = _limelight.getHorizontalAngle();
                        double yaw = _imu.getYaw();

                        // Find the pose of the robot _when the picture was taken_
                        long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
                        BasicPose pose = (BasicPose)_poseTracker.get(timeKey);

                        // Get the angle from the pose if one was found--otherwise use yaw
                        double poseAngle = pose == null ? yaw : pose.getAngle();

                        // Now adjust the limelight angle based on the change in yaw from when the picture was taken to now
                        double offsetCompensation = yaw - poseAngle;
                        _visionAngle = limeLightAngle - offsetCompensation;
                        _useVisionAngle = true;
                        metric("VisionAngle", _visionAngle);
                    } else {
                        _state = State.tooclose;
                        _limelight.disableLEDs();
                    }
                }
                break;
        }
        double leftDistance = _driveTrain.getLeftDistance() * _direction;
        double rightDistance = _driveTrain.getRightDistance() * _direction;
        double leftSpeed = _leftFollower.calculate(leftDistance) * _direction;
        double rightSpeed = _rightFollower.calculate(rightDistance) * _direction;
        double yaw = _imu.getYaw();
        double heading = _backwards ? Pathfinder.boundHalfDegrees(yaw + 180) : yaw;
        //double heading = yaw;

        double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(_leftFollower.getHeading()));
        double headingDifference = Pathfinder.boundHalfDegrees((_useVisionAngle ? _visionAngle : desiredHeading) - heading);
        double turn =  Constants.AutoDrivePath.K_TURN * (1.0/80.0) * headingDifference;

/*        metric("LeftDistance",leftDistance);
        metric("RightDistance", rightDistance);
        metric("LeftSpeed",leftSpeed);
        metric("RightSpeed", rightSpeed);
        metric("Yaw", yaw);
        metric("Heading", heading);
        metric("DesiredHeading", desiredHeading);
        metric("HeadingDifference", headingDifference);
        metric("Turn", turn);

        metric("LeftOutput",leftSpeed + turn);
        metric("RightOutput", rightSpeed - turn);
*/
        _driveTrain.setPower(leftSpeed + turn, rightSpeed - turn, true);
    }

    @Override
    protected boolean isFinished() {
        if (_leftFollower.isFinished() || _rightFollower.isFinished()) {
            info("AutoDrivePath finished");
            return true;
        }
        return false;
    }

    @Override
    protected void end() {
        super.end();
        _leftFollower.reset();
        _rightFollower.reset();
        if (_pathNotifier != null) { _pathNotifier.stop(); }
        _limelight.disableLEDs();
        info("Ending AutoDrivePath");
    }

    private enum State {
        normal,
        tracking,
        tooclose
    }
}
