package frc.robot.settings;

public final class RobotMap {
    public static final class ROBOT {
        public static class INTAKE {
            public static class CAN {
                public static final int MOTOR = 16;
            }

            public static class DIO {
                public static final int BREAK_BEAM = 0;
            }
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
