package frc.robot.settings;

public final class RobotMap {
    public static final class ROBOT {
        public static final class ELEVATOR {
            public static final int kLeadMotorPwmPort = 0;
            public static final int kFollowerMotorPwmPort = 1;
    
            public static final int kEncoderAChannel = 0;
            public static final int kEncoderBChannel = 1;
        }

        public static class CLIMBER {
            public static class CAN {
                public static final int LEFT_MOTOR = 19;
                public static final int RIGHT_MOTOR = 20;
            }
        }

        public static final class DRIVE_STATION {
            public static final int DRIVER_USB_XBOX_CONTROLLER = 0;
            public static final int OPERATOR_USB_XBOX_CONTROLLER = 1;
        }

        public static final class LEDS {
            public static final int PWM_PORT = 0;
        }
    }
}
