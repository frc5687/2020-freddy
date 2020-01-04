package org.frc5687.freddy2020.robot;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and then in numerical order.
     * Note that for CAN, ids must be unique per device type, but not across types.
     * Thus, you cannot have two SparkMax controllers with Id 0, but you can have a SparkMax with Id 0 and a TalonSRX with Id 0.
      */
    public static class CAN {

        /*  Example:
                        public static final int LEFT_MASTER_SPARK= 1;
                        */
        public static class SPARKMAX {
            public static final int DRIVE_LEFT_MASTER =  11;
            public static final int DRIVE_RIGHT_MASTER = 6;
            public static final int DRIVE_LEFT_FOLLOWER = 14;
            public static final int DRIVE_RIGHT_FOLLOWER = 8;
            public static final int ELEVATOR_MOTOR = 5;
            public static final int RIGHT_ARM = 16;
            public static final int LEFT_ARM = 9;
            public static final int STILT = 13;
        }
        public static class TALONSRX{
            public static final int ROLLER = 1;
        }
    }

    /**
     * There should be an entry here for each PWM port, preferrably in numerical order.
     * Note that for PWM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PWM {
        public static final int LeftBlinkin = 0;
        public static final int RightBlinkin = 1;
        public static final int Wheelie = 2;
        /*  Example:
        public static final int ARM_VICTORSP = 0;
        */
    }

    /**
     * There should be an entry here for each PCM port, preferrably in numerical order.
     * Note that for PCM only one device can connect to each port, so the numbers should be unique.
     */
    public static class PCM {
        /* Example:
        public static final int LEFT_PINCER_OPEN = 5;
        */

        //PCM ports are not in the robot yet, using 0 and 1.
        public static final int WRIST_DOWN = 1;
        public static final int WRIST_UP = 2;
        public static final int SHIFTER_HIGH = 7;
        public static final int SHIFTER_LOW = 6;
        public static final int CLAW_OPEN = 4;
        public static final int CLAW_CLOSE = 5;
        public static final int CLAW_WRIST_UP = 0;
        public static final int CLAW_WRIST_DOWN = 3;
    }

    /**
     * There should be an entry here for each PDP breaker, preferrably in numerical order.
     * Note that only on device can be connected to each breaker, so the numbers should be unique.
     */
    public static class PDP {
        /* Example:
        public static final int ARM_VICTORSP = 0;
        */
        public static final int GRIPPER_VACCUUM = 8;
        public static final int RIGHT_ARM_COMP = 1;
        public static final int LEFT_ARM_COMPO = 14;
        public static final int RIGHT_ARM_PRACTICE = 3;
        public static final int LEFT_ARM_PRACTICE = 2;
        public static final int DRIVE_LEFT_MASTER = 12;
        public static final int DRIVE_LEFT_FOLLOWER = 13;
        public static final int DRIVE_RIGHT_MASTER = 0;
        public static final int DRIVE_RIGHT_FOLLOWER = 2;
    }

    /**
     * There should be an entry here for each Analgo port, preferrably in numerical order.
     * Note that for Analog only one device can connect to each port, so the numbers should be unique.
     */
    public static class Analog {
        /*
        public static final int ARM_POTENTIOMETER = 7;
         */
        public static final int FRONT_IR = 0;
        public static final int BALL_IR = 1;
        public static final int DOWN_IR = 2;
        public static final int POSITION_SWITCH = 4;
        public static final int MODE_SWITCH = 5;
    }

    /**
     * There should be an entry here for each DIO port, preferrably in numerical order.
     * Note that for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ELEVATOR_A = 0;
        public static final int ELEVATOR_B = 1;
        public static final int ELEVATOR_TOP_HALL = 2;
        public static final int ELEVATOR_BOTTOM_HALL = 3;

        public static final int STILT_EXTENDED_HALL = 5;
        public static final int STILT_RETRACTED_HALL = 4;
        public static final int STILT_MIDDLE = 10;

        public static final int ARM_RIGHT_STOWED_HALL = 6;
        public static final int ARM_LEFT_STOWED_HALL = 7;

        public static final int HATCH_DETECTION_LIMIT = 9;
        public static final int SHOCK_HALL = 8;

        public static final int DRIVE_RIGHT_B = 13;
        public static final int DRIVE_RIGHT_A = 19;
        public static final int DRIVE_LEFT_B = 21;
        public static final int DRIVE_LEFT_A = 23;



        /* Example:
        public static final int ARM_FRONT_LIMIT = 0;
        */
    }

}
