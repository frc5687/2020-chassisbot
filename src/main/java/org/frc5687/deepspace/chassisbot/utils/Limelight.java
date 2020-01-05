package org.frc5687.deepspace.chassisbot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.deepspace.chassisbot.Constants;
import java.util.ArrayList;

import static org.frc5687.deepspace.chassisbot.Constants.Limelight.*;

public class Limelight extends OutliersProxy {
    NetworkTable _table;
    NetworkTableEntry _tx;
    NetworkTableEntry _ty;
    NetworkTableEntry _tv;
    NetworkTableEntry _ta;
    NetworkTableEntry _ts;
    NetworkTableEntry _tl;
    NetworkTableEntry _tshort;
    NetworkTableEntry _tlong;
    NetworkTableEntry _tvert;
    NetworkTableEntry _thor;
    NetworkTableEntry _getpipe;
    NetworkTableEntry _camtran;

    NetworkTableEntry _ledmode;
    NetworkTableEntry _cammode;
    NetworkTableEntry _pipeline;
    NetworkTableEntry _stream;

    public Limelight() {
        this("limelight");
    }

    public Limelight(String key) {
        _table = NetworkTableInstance.getDefault().getTable(key);
        _tx = _table.getEntry("tx");
        _ty = _table.getEntry("ty");
        _tv = _table.getEntry("tv");
        _ta = _table.getEntry("ta");
        _ts = _table.getEntry("ts");
        _tl = _table.getEntry("tl");
        _tshort = _table.getEntry("tshort");
        _tlong = _table.getEntry("tlong");
        _tvert = _table.getEntry("tvert");
        _thor = _table.getEntry("thor");
        _getpipe = _table.getEntry("getpipe");
        _camtran = _table.getEntry("camtran");

        _ledmode = _table.getEntry("ledMode");
        _cammode = _table.getEntry("camMode");
        _pipeline = _table.getEntry("pipeline");
        _stream = _table.getEntry("stream");
    }
    public void enableVision() {
        _cammode.setNumber(0);
    }

    public void disableVision() {
        _cammode.setNumber(1);
    }

    public void defaultLEDs() {
        _ledmode.setNumber(0);
    }

    public void enableLEDs() {
        _ledmode.setNumber(3);
    }

    public void disableLEDs() {
        _ledmode.setNumber(1);

    }
    public void blinkLEDs() {
        _ledmode.setNumber(2);
    }

    public void setPipeline(Pipeline pipeline) {
        _pipeline.setNumber(pipeline.getValue());
    }

    public void setPipeline(int pipeline) {
        _pipeline.setNumber(pipeline);
    }

    public void setStreamingMode(StreamMode mode) {
        _stream.setNumber(mode.getValue());
    }

    public boolean isTargetSighted() {
        return _tv.getDouble(0) > 0;
    }

    public double getHorizontalAngle() {
        return _tx.getDouble(0.0);
    }

    public double getVerticalAngle() { return _ty.getDouble(0);}

    public double getTargetArea() { return _ta.getDouble(0); }

    public double getLatency() {
        return _tl.getDouble(0) + Constants.Limelight.OVERALL_LATENCY_MILLIS;
    }

    public double getCamTran(int variable) {
        double[] camtranData = _camtran.getDoubleArray(new double[]{});
        /**
         *         x = camtranData[0];
         *         y = camtranData[1];
         *         z = camtranData[2];
         *         pitch = camtranData[3];
         *         yaw = camtranData[4];
         *         roll = camtranData[5];
         */
        if (variable > camtranData.length) {
            error("variable " + variable + " out of range " + camtranData.length);
            return 0; }
        return camtranData[variable];
    }



    public double getTargetDistanceFromTA() {
        double distance = 82.819*(Math.exp(-0.206*getTargetArea()));
        return distance;

    }
    public double getTargetDistance() {
        double heightOffset = (LIMELIGHT_HEIGHT - TARGET_HEIGHT);
        double limeLightYAngle = getVerticalAngle();
        double angleY = (LIMELIGHT_ANGLE - limeLightYAngle);
        double tanY = Math.tan(angleY * (Math.PI / 180));
        double distance = (heightOffset)/tanY;
        return distance;
    }

    @Override
    public void updateDashboard() {
        metric("tx",_tx.getDouble(0.0));
        metric("ty",_ty.getDouble(0.0));
        metric("tv", _tv.getDouble(0.0));
        metric("DistanceFromTarget", getTargetDistanceFromTA());
        metric("Distance", getTargetDistance());
//        metric("CamTranX", getCamTran(0));
//        metric("CamTranY", getCamTran(1));
//        metric("CamTranZ", getCamTran(2));
//        metric("Pitch", getCamTran(3));
//        metric("Yaw", getCamTran(4));
//        metric("Roll", getCamTran(5));
    }

    public boolean isTargetCentered() {
        return (isTargetSighted() && Math.abs(getHorizontalAngle()) < Constants.Auto.Align.TOLERANCE);
    }

    public boolean areLEDsOn() {
        return _ledmode.getNumber(0).doubleValue() == 3.0;
    }

    public enum StreamMode {
        SIDE_BY_SIDE(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);

        private int _value;

        StreamMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public enum Pipeline {
        TapeTrackingLargest(0),
        TapeTrackingClosest(1),
        TapeTrackingHighest(2),
        CargoTrackingLargest(8),
        CargoTrackingClosest(9);

        private int _value;

        Pipeline(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

}
