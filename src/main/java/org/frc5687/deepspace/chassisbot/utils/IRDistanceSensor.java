package org.frc5687.deepspace.chassisbot.utils;

import edu.wpi.first.wpilibj.AnalogInput;

public class IRDistanceSensor extends AnalogInput {
    private double _coefficient;
    private double _power;

    public IRDistanceSensor(int channel, Type type) {
        super(channel);
        _coefficient = _coefficients[type.getValue()];
        _power = _powers[type.getValue()];
    }


    public double getRaw() {
        return super.getAverageVoltage();
    }

    public double getDistance() {
        return (_coefficient * Math.pow(getRaw(), _power)) / 2.54;

    }
    @Override
    public double pidGet() {
        return getDistance();
    }



    public enum Type {
        VERYSHORT(0), //
        SHORT(1),     // 4-30cm
        MEDIUM(2),    // 10-80cm
        LONG(3),      // 15-150cm
        ULTRALONG(4); // 100-500cm

        private int _value;

        Type(int value) {
            _value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    /**
     * a in the voltage-to-distance equation distance = a * voltage ^ b
     */
    private static final double[] _coefficients =  {
            0,
            27.385,
            23.999,
            0,
            0
    };

    /**
     * b in the voltage-to-distance equation distance = a * voltage ^ b
     */
    private static final double[] _powers = {
            0,
            -1.203  ,
            -1.145,
            0,
            0
    };

}
