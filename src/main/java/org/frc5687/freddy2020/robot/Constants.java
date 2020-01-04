package org.frc5687.freddy2020.robot;

public class Constants {
    /**
     *
     */
    public static final int CYCLES_PER_SECOND = 50;
    public static final int TICKS_PER_UPDATE = 100;
    public static final double METRIC_FLUSH_PERIOD = 1.0;
    public static final int TICKS_PER_UPDATE_COMP = 100;
    public static final double FINAL_WARNING = 15;

    public class DriveTrain {

        public static final double DEADBAND = 0.25;
        public static final double SPEED_SENSITIVITY = 0.9;
        public static final double ROTATION_SENSITIVITY = 0.75;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 0;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0;
        public static final double TURNING_SENSITIVITY_HIGH_GEAR = 0;
        public static final double TURNING_SENSITIVITY_LOW_GEAR = 0;

        public static final double CREEP_FACTOR = 0.25;
        public static final double LEFT_DISTANCE_PER_PULSE = 0.0286206896551724;
        public static final double RIGHT_DISTANCE_PER_PULSE = 0.0286206896551724;


        public static final boolean LEFT_MOTORS_INVERTED = true;
        public static final boolean RIGHT_MOTORS_INVERTED = false;

        public static final long LOCK_TIME = 80;
        public static final long DROPOUT_TIME = 100;
        public static final long SEEK_TIME = 500;


        public static final double MAX_SPEED_IPS = 156.0;
        public static final double CAP_SPEED_IPS = .8 * MAX_SPEED_IPS;
        public static final double MAX_ACCELERATION_IPSS = CAP_SPEED_IPS / 2;
        public static final double MAX_JERK_IPSSS = CAP_SPEED_IPS;
        public static final double RAMP_RATE = 0.125;
        public static final int STALL_CURRENT_LIMIT = 50;
        public static final int FREE_CURRENT_LIMIT = 60;
        public static final double SECONDARY_LIMIT = 90;


        public static final double SLOW_ZONE_COMP = 30;
        public static final double SLOW_ZONE_PROTO = 35;

        public static final double MEDIUM_ZONE_COMP = 70;
        public static final double MEDIUM_ZONE_PROTO = 115;

        public static final double SLOW_SPEED_COMP = 0.4;
        public static final double SLOW_SPEED_PROTO = 0.5;

        public static final double MEDIUM_SPEED_COMP = 0.6;
        public static final double MEDIUM_SPEED_PROTO = 0.6;
    }
    public class Intake {
        //Roller
        public static final double HIGH_POW = 1.0;
        public static final double LOW_POW = -HIGH_POW;
        public static final double ROLLER_SPEED = 0.7;
        public static final double MAX_ROLLER_SPEED = 1.0;
        public static final boolean MOTOR_INVERTED = true;
        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY = 0.5;
        public static final long ROLLER_TIME_MILLI_SEC = 500;
        public static final int CARGO_DETECTED_THRESHOLD = 800;
        //Wrist
        public static final long RAISE_WRIST_MILLI_SEC = 250;
        public static final long LOWER_WRIST_MILLI_SEC = 250;
        //Claw
        public static final long OPEN_CLAW_MILLI_SEC = 40;
        public static final long CLOSE_CLAW_MILLI_SEC = 40;
        public static final long CLOSE_CLAW_MILLI_SS = 40; // Delay in sandstorm
        public static final long CLAW_RAISE_WRIST_MILLI_SEC = 250;
        public static final long CLAW_LOWER_WRIST_MILLI_SEC = 250;


        public static final long CARGO_EJECT_MILLIS = 200;
        public static final double CARGO_EJECT_SPEED = -0.5;

        public static final long SCORE_ROLLER_MILLIS = 120;
        public static final long SCORE_KICK_MILLIS = 120;
    }
    public class Shifter {
        public static final long STOP_MOTOR_TIME = 60;
        public static final long SHIFT_TIME = 60;

        public static final double SHIFT_UP_THRESHOLD = 50; // in inches per second graTODO tune
        public static final double SHIFT_DOWN_THRESHOLD = 40; // in inches per second TODO tune

        public static final long AUTO_WAIT_PERIOD = 500;
        public static final long MANUAL_WAIT_PERIOD = 3000;

    }

    public static class Elevator {
        public static final double MAX_SPEED = 1.0;
        public static final double MAX_SPEED_UP = 1.0;
        public static final double MAX_SPEED_DOWN = 1.0;
        public static final double SPEED_UP = 1.0;
        public static final double SPEED_DOWN = 1.0;

        public static final long CREEP_TIME = 200;

        public static final double JELLO_SPEED_UP = 0.3;
        public static final double JELLO_SPEED_DOWN = 0.3;

        public static final double TOP_JELLO_ZONE = 500;
        public static final double BOTTOM_JELLO_ZONE = 1200;

        public static final double DEADBAND = 0.1;
        public static final double SENSITIVITY = 0.5;
        public static final boolean ELEVATOR_MOTOR_INVERTED = true;
        public static final int TOLERANCE = 8;
        public static final double MAX_VELOCITY_IPS = 27.0;
        public static final double TICKS_PER_INCH = 111.1111111;
        public static final double STEPS_UP = 10; //10;
        public static final double STEPS_DOWN = 30; //30;
        public static final double TICKS_PER_STEP_COMP = 100;
        public static final double TICKS_PER_STEP_PROTO = 50;
        public static final double MIN_SPEED = 0.3;
        public static final double GOAL_SPEED = 0.5;
        public static final int BOTTOM_CAM_ZONE = 450;
        public static final double SANDSTORM_PICKUP_SPEED = 0.5;
        public static final double MODE_SPEED = 0.6;

        public static class PID {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
        public static class Path {
            public static final double kP = 1.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
    }
    public static class OI {
        public static final double AXIS_BUTTON_THRESHHOLD = 0.2;
        public static final long RUMBLE_MILLIS = 250;
        public static final double RUMBLE_INTENSITY = 1.0;
        public static final long RUMBLE_PULSE_TIME = 100;
        public static final int KILL_ALL = 4;
        public static final int OVERRIDE = 8;
        public static final int RED_CHANNEL = 2;
        public static final int GREEN_CHANNEL = 3;
        public static final int BLUE_CHANNEL = 1;
    }

    public static class Arm {
        public static final double SENSITIVITY = 0.5;
        public static final double DEADBAND = 0.8;

        public static final double MAX_DRIVE_SPEED = 1;
        public static final double HOLD_SPEED = 0.01;
        public static final double STOW_SPEED = -0.2;

        public static final boolean LEFT_MOTOR_INVERTED = true;
        public static final boolean RIGHT_MOTOR_INVERTED = false;

        public static final int CLIMB_STALL_LIMIT = 30;
        public static final int CLIMB_FREE_LIMIT = 80;
        public static final int STOW_STALL_LIMIT = 10;
        public static final int STOW_STALL_THRESHOLD = 8;

        public static final double kI = 0;
        public static final double kP = 0.1;
        public static final double kD = 0;
        public static final double TOLERANCE = 2.0;
        public static final double SPEED = 1;
        public static final double SPEED_UP = 1;
        public static final double SPEED_DOWN = 1;

        public static final double DEGREES_PER_TICK = 90.0 / 70.0;

        public static final double STOWED_ANGLE = 0.0;
        public static final long ENCODER_ZERO_TIMEOUT = 10000000;
    }

    public class Lights {
        public static final double SOLID_BLUE = 0.87;
        public static final double PULSING_BLUE = -0.09;
        public static final double BEATING_BLUE = 0.23;

        public static final double SOLID_RED = 0.61;
        public static final double PULSING_RED = -0.11;
        public static final double BEATING_RED = 0.25;

        public static final double SOLID_GREEN = 0.77;
        public static final double PULSING_GREEN = 0.77; // replace
        public static final double BEATING_GREEN = 0.00; // unused

        public static final double SOLID_PURPLE = 0.91;
        public static final double PULSING_PURPLE = 0.05;
        public static final double BEATING_PURPLE = 0.00;

        public static final double SOLID_ORANGE = 0.06;
        public static final double PULSING_ORANGE = 0.07;
        public static final double BEATING_ORANGE = 0.08;

        public static final double SOLID_YELLOW = 0.69;
        public static final double PULSING_YELLOW = 0.10;
        public static final double BEATING_YELLOW = 0.11;

        public static final double SOLID_BLACK = 0.99;

        public static final double SOLID_HOT_PINK = 0.57;

        public static final double CONFETTI = -0.87;

        public static final double BLEND_1 = -0.03;
        public static final double SCANNING_1 = -0.01;
        public static final double CHASING_1 = 0.01;
        public static final double SLOW_BEAT_1 = 0.03;
        public static final double MEDIUM_BEAT_1 = 0.05;
        public static final double FAST_BEAT_1 = 0.07;
        public static final double BREATH_SLOW_1 = 0.09;
        public static final double BREATH_FAST_1 = 0.11;
        public static final double SHOT_1 = 0.13;
        public static final double STROBE_1 = 0.15;

        public static final double BLEND_2 = 0.17;
        public static final double SCANNING_2 = 0.19;
        public static final double CHASING_2 = 0.21;
        public static final double SLOW_BEAT_2 = 0.23;
        public static final double MEDIUM_BEAT_2 = 0.25;
        public static final double FAST_BEAT_2 = 0.27;
        public static final double BREATH_SLOW_2 = 0.29;
        public static final double BREATH_FAST_2 = 0.31;
        public static final double SHOT_2 = 0.33;
        public static final double STROBE_2 = 0.35;


        public static final double WHITE_SHOT = -0.81;
        public static final double SOLID_WHITE = 0.93;
    }
    public class Stilt {
        public static final boolean MOTOR_INVERTED = false;
        public static final double SENSITVITY = 0.1;
        public static final double DEADBAND = 0.1;
        public static final double MAX_UP_SPEED= 0.5;
        public static final double MAX_DOWN_SPEED = 0.4;
        public static final double STILT_HOLD_SPEED = 0.07;
        public static final double TOLERANCE=5.0;
        public static final double MIDDLE_POSITION=25.0;
        public static final double BOTTOM_POSITION=0.0;
        public static final double TOP_POSITION=40.0;
        public static final double DOWN_IR_THRESHOLD = 1500.0;
        public static final double DOWN_IR_THRESHOLD_LOW = 1000.0;
        public static final double TILT_THRESHOLD =  2100.0;
    }
    public class Limelight {
        public static final double TARGET_HEIGHT = 29;
        public static final double LIMELIGHT_HEIGHT = 41.5;
        public static final double LIMELIGHT_ANGLE = 20;
        public static final double OVERALL_LATENCY_MILLIS = 11;
    }


    /*
     There should be a nested static class for each subsystem and for each autonomous command that needs tuning constants.
     For example:
    public static class DriveTrain {
        public static final double DEADBAND = 0.3;
        public static final double SENSITIVITY_LOW_GEAR = 0.8;
        public static final double SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY = 1.0;
        public static final double ROTATION_SENSITIVITY_HIGH_GEAR = 1.0;
        public static final double ROTATION_SENSITIVITY_LOW_GEAR = 0.8;
    }
     */


    public static class Auto {
        public static final double MIN_IMU_ANGLE = -180.0;
        public static final double MAX_IMU_ANGLE = 180.0;

        public static final double MAX_PITCH = 20.0;
        public static final double MAX_ROLL = 20.0;
        public static final double IR_THRESHOLD = 24.0;
        public static final long AUTOCHOOSER_DELAY = 5000;

        public static class Climb {
            public static final double ENDGAME_CUTOFF = 30.0;

            public static final double STILT_TILT_HOLD_SPEED =0.1;
            public static final double TILT_PITCH = 11.0;


            public static final double STILT_SPEED = 1.0; // .55
            public static final double STILT_TILT_SPEED = 0.5; // .55
            public static final double STILT_HIGH_HOLD_SPEED = 0.4;
            public static final double STILT_LOW_HOLD_SPEED = 0.1;
            public static final double RAISE_STILT_SPEED = -0.5; // TODO: -0.3

            public static final double ARM_SPEED = 0.65;
            public static final double ARM_SLOW_SPEED = 0.2;
            public static final double ARM_HOLD_SPEED = 0.0;
            public static final double RAISE_ARM_SPEED = -0.2;

            public static final double H3_CONTACT_ANGLE = 95.0;
            public static final double H2_CONTACT_ANGLE = 145.0;
            public static final double H3_SLOW_ANGLE = 165.0;
            public static final double H2_SLOW_ANGLE = 165.0;
            public static final double H3_BOTTOM_ANGLE = 180.0;
            public static final double H2_BOTTOM_ANGLE = 180.0;

            public static final double ARM_RETRACT_ANGLE = 165.0;

            public static final double INITIAL_ARM_SPEED = 0.6;

            public static final double WHEELIE_FORWARD_SPEED_HIGH = 1.0;
            public static final double WHEELIE_FORWARD_SPEED_LOW = 0.73;

            public static final double DRIVE_FORWARD_SPEED = 0.1;

            public static final double PARK_SPEED = 0.3;
            public static final double PARK_DISTANCE = 18;
            public static final long PARK_TIMEOUT = 3000;

            public static final long STILT_TIMEOUT_HIGH = 4000;
            public static final long STILT_TIMEOUT_LOW = 250;

        }

        public static class AutoDrivePath {
            public static final double ANGLE_TOLERANCE = 0.25;
            public static final double kPangle = .001;
            public static final double kIangle = .0001;
            public static final double kDangle = .001;

            public static final double kPdist = .001;
            public static final double kIdist = .0001;
            public static final double kDdist = .001;
        }

        public class DriveToTarget {

            public static final double TURN_SPEED = 0.15;

            public static final double kPAngle = 0.015;
            public static final double kIAngle = 0.00;
            public static final double kDAngle = 0.5;

            public static final double kPDistance = 0.2;
            public static final double kIDistance = 0.000;
            public static final double kDDistance = 0.3;

            public static final double ANGLE_TOLERANCE = 1;
            public static final double DISTANCE_TOLERANCE = 1;

            public static final double MAX_SPEED = .7;
            public static final double DESIRED_TARGET_AREA = 5;
            public static final double STOP_DISTANCE = 24.00;
        }
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
                public static final double IPS = 130; // Inches Per Second
            }

            public class MaxAcceleration {
                public static final double METERS = 2; // Meters Per Second Squared
                public static final double INCHES = 80.0;
            }

            public class MaxJerk {
                public static final double METERS = 6.0; // Meters Per Second Cubed
                public static final double INCHES = 200.0;
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

    public class RotarySwitch {
        public static final double TOLERANCE = 0.02;
    }

    public class AutoDrivePath {
        public static final double K_TURN = 0.2;
    }
}