package org.frc5687.deepspace.chassisbot.commands.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.deepspace.chassisbot.Constants;
import org.frc5687.deepspace.chassisbot.commands.OutliersCommand;
import org.frc5687.deepspace.chassisbot.subsystems.SparkMaxDriveTrain;

public class AutoAlign extends OutliersCommand implements PIDOutput {
    private PIDController _controller;
    private double _angle;
    private double _speed;
    private long _timeout = 2000;

    private double _pidOut;

    private long _onTargetSince;
    private long _startTimeMillis;
    private long _endTimeMillis;
    private boolean _aborted = false;

    private SparkMaxDriveTrain _driveTrain;
    private AHRS _imu;

    private String _state = "";


    private double _tolerance;

    public AutoAlign(SparkMaxDriveTrain driveTrain, AHRS imu, double angle, double speed, long timeout, double tolerance, String state) {
        requires(driveTrain);
        _angle = angle;
        _speed = speed;
        _driveTrain = driveTrain;
        _imu = imu;
        _timeout = timeout;
        _tolerance = tolerance;
        _state = state;
        //logMetrics("Angle","pidOut","imu","onTarget");
    }

    @Override
    protected void initialize() {
        error("Starting AutoAlign");
        super.initialize();
        double kP = Constants.Auto.Align.kP; // Double.parseDouble(SmartDashboard.getString("DB/String 0", ".04"));
        double kI = Constants.Auto.Align.kI; // Double.parseDouble(SmartDashboard.getString("DB/String 1", ".006"));
        double kD = Constants.Auto.Align.kD; //Double.parseDouble(SmartDashboard.getString("DB/String 2", ".09"));

        _controller = new PIDController(kP, kI, kD, _imu, this, 0.01);
        _controller.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _controller.setOutputRange(-_speed, _speed);
        _controller.setAbsoluteTolerance(_tolerance);
        _controller.setContinuous();
        _controller.setSetpoint(_angle);
        _controller.enable();
        info(_state + " initialized to " + _angle + " at " + _speed);
        info("kP="+kP+" , kI="+kI+", kD="+kD + ",T="+ Constants.Auto.Align.TOLERANCE);
        _startTimeMillis = System.currentTimeMillis();
        _endTimeMillis = _startTimeMillis + _timeout;
        metric("Angle", _angle);
    }

    @Override
    protected void execute() {
        actOnPidOut();
        // Check pitch and tilt
        double pitch = _imu.getPitch();
        double roll = _imu.getRoll();

        if (Math.abs(pitch) > Constants.Auto.MAX_PITCH) {
            error("Excessive pitch detected (" + pitch + ")");
            _controller.disable();
            _aborted = true;
        }

        if (Math.abs(roll) > Constants.Auto.MAX_ROLL) {
            error("Excessive roll detected (" + roll + ")");
            _controller.disable();
            _aborted = true;
        }

        metric("onTarget", _controller.onTarget());
        metric("imu", _imu.getYaw());
    }

    private void actOnPidOut() {
        if (_pidOut > 0 && _pidOut < Constants.Auto.Align.MINIMUM_SPEED) {
            _pidOut = Constants.Auto.Align.MINIMUM_SPEED;
        }
        if (_pidOut < 0 && _pidOut > -Constants.Auto.Align.MINIMUM_SPEED) {
            _pidOut = -Constants.Auto.Align.MINIMUM_SPEED;
        }
        _driveTrain.setPower(_pidOut, -_pidOut, true);

        metric("pidOut", _pidOut);
        metric("Angle", _angle);
    }

    @Override
    protected boolean isFinished() {
        if (_aborted) { return true; }
        if (!_controller.onTarget()) {
            _onTargetSince = 0;
        }

        if(System.currentTimeMillis() >= _endTimeMillis){
            debug("AutoAlign timed out after " + _timeout + "ms at " + _imu.getYaw());
            return true;
        }

        if (_controller.onTarget()) {
            if (_onTargetSince == 0) {
                info("AutoAlign reached target " + _imu.getYaw());
                _onTargetSince = System.currentTimeMillis();
            }
        }

        return false;
    }

    @Override
    protected void end() {
        super.end();
        _driveTrain.setPower(0,0, true);
        info("AutoAlign finished: angle = " + _imu.getYaw() + ", time = " + (System.currentTimeMillis() - _startTimeMillis));
        _controller.disable();
        debug("AutoAlign.end() controller disabled");
    }

    @Override
    public void pidWrite(double output) {
        _pidOut = output;
        actOnPidOut();
    }


    /*
    Used in AutoAlign to select which types of turns to do.
    bothSides will turn normally, or "in place" (hah!)
    leftOnly will use the pidOut to drive the left side of the drivetrain, but the right side will be in 0in/s talon velocity pid mode.
    rightOnly will use the pidOut to drive the right side of the drivetrain, but the left side will be in 0in/s talon velocity pid mode.
     */


}

