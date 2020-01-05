package org.frc5687.deepspace.chassisbot;

public class Constants {
    /**
     *
     */
    public static final int CYCLES_PER_SECOND = 50;
    public static final int TICKS_PER_UPDATE = 10;
    public static final double METRIC_FLUSH_PERIOD = 1.0;

    public class DriveTrain {
        public static final double DEADBAND = 0.1;
        public static final double SPEED_SENSITIVITY = 0.80;
        public static final double ROTATION_SENSITIVITY = 0.75;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 1;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 1;
        public static final double TURNING_SENSITIVITY_HIGH_GEAR = 1;
        public static final double TURNING_SENSITIVITY_LOW_GEAR = 1;

        public static final double CREEP_FACTOR = 0.25;
        public static final double LEFT_DISTANCE_PER_PULSE = 0.02526315789473684210526315789474;
        public static final double RIGHT_DISTANCE_PER_PULSE = 0.02526315789473684210526315789474;

        public static final boolean LEFT_FRONT_MOTOR_INVERTED = true;
        public static final boolean LEFT_BACK_MOTOR_INVERTED = true;
        public static final boolean RIGHT_FRONT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_BACK_MOTOR_INVERTED = false;
        public static final double TURNING_SENSITIVITY = 0.5;


        public static final long LOCK_TIME = 80;
        public static final long DROPOUT_TIME = 100;
        public static final long SEEK_TIME = 500;

        public static final double MAX_SPEED_IPS = 168.0;
        public static final double CAP_SPEED_IPS = .8 * MAX_SPEED_IPS;
        public static final double MAX_ACCELERATION_IPSS = CAP_SPEED_IPS / 2;
        public static final double MAX_JERK_IPSSS = CAP_SPEED_IPS;
        public static final double RAMP_RATE = 0.125;
        public static final int STALL_CURRENT_LIMIT = 50;
        public static final int FREE_CURRENT_LIMIT = 60;
        public static final double SECONDARY_LIMIT = 90;

        public static final double SLOW_ZONE = 30;
        public static final double MEDIUM_ZONE = 70;
        public static final double SLOW_SPEED = 0.4;
        public static final double MEDIUM_SPEED = 0.6;
    }

    public class Turret {
        public static final double DEADBAND = 0.1;
        public static final boolean TURRET_MOTOR_INVERTED = true;
        public static final double MAX_SPEED = 0.8;
        public static final double MAX_FORWARD_SPEED = MAX_SPEED;
        public static final double MAX_REVERSE_SPEED = -MAX_SPEED;
        public static final double DEGREES_PER_TICK = 0.0;
        public static final double MAX_TURRET_ANGLE = 360; // in degrees
        public static final double MID_TURRET_ANGLE = 180; // in degrees
        public static final double MIN_TURRET_ANGLE = 0; // in degrees

        public static final double ROTATION_K = 0.03;

        public static final long LOCK_TIME = 80;
        public static final double TOLERANCE = 1;

        public static final double TICKS_PER_DEGREES = 0.2014611111111111111111111111111;
    }

    public class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in inches per second graTODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; // in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;
    }


    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
        public static final long RUMBLE_PULSE_TIME = 100;
        public static final int KILL_ALL = 4;
        public static final int OVERRIDE = 8;
    }

    public class Auto {
        public static final double MIN_IMU_ANGLE = -180.0;
        public static final double MAX_IMU_ANGLE = 180.0;

        public static final double MAX_PITCH = 20.0;
        public static final double MAX_ROLL = 20.0;
        public static final double IR_THRESHOLD = 24.0;


        public class Align {
            public static final double SPEED = 0.15;

            public static final double kP = 0.03; //0.03;
            public static final double kI = 0.000; // 0;.000.1
            public static final double kD = 0.3;  //0.1;
            public static final double TOLERANCE = 1; // 0.5
            public static final double MINIMUM_SPEED = 0;//0.15;
            /*
             *time the angle must be on target for to be considered steady
             */
            public static final double STEADY_TIME = 60;
            public static final double STEER_K = .015;
        }

        public class Drive {
            public static final double SPEED = 1.0;

            public static final double MIN_SPEED = 0.25;
            public static final double MIN_TRACK_DISTANCE = 18;
            public static final int MAX_GARBAGE = 5;

            public class MaxVel {
                public static final double MPS = 2.33; // Meters Per Second
                public static final double IPS = 160; // Inches Per Second
            }

            public class MaxAcceleration {
                public static final double METERS = 2; // Meters Per Second Squared
                public static final double INCHES = 640.0;
            }

            public class MaxJerk {
                public static final double METERS = 6.0; // Meters Per Second Cubed
                public static final double INCHES = 2000.0;
            }

            public static final long STEADY_TIME = 100;
            public static final long ALIGN_STEADY_TIME = 100;


            public class AnglePID {
                public static final double kP = 0.01;
                public static final double kI = 0.000;
                public static final double kD = 0.00;
                public class kV {
                    public static final double MPS = 1.0 / MaxVel.MPS;
                    public static final double IPS = 1.0 / MaxVel.IPS;
                }
                public static final double PATH_TURN = 0.4; // 1.0
                public static final double MAX_DIFFERENCE = 0.05;
                public static final double TOLERANCE = .25;
            }
        }
    }
    public class Limelight {
        public static final double TARGET_HEIGHT = 29;
        public static final double LIMELIGHT_HEIGHT = 33.5;
        public static final double LIMELIGHT_ANGLE = 0;
        public static final double OVERALL_LATENCY_MILLIS = 11;
    }

    public class AutoDrivePath {
        public static final double K_TURN = 0.2;
    }

    public class RotarySwitch {
        public static final double TOLERANCE = 0.02;
    }


    /*
     There should be a nested static class for each subsystem and for each autonomous command that needs tuning constants.
     For example:
    public static class VictorSPDriveTrain {
        public static final double DEADBAND = 0.3;
        public static final double SENSITIVITY_LOW_GEAR = 0.8;
        public static final double SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY = 1.0;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0.8;
    }
     */

}