package frc.robot.settings;

public final class RobotMap {
    public static final class ROBOT {
        public static final class CoralSystem {
            public static final class ElevatorMap {
                public static final int leadMotorCANID = 5;
                public static final int followerMotorCANID = 6;

            }

            public static final class PivotMap {
                public static final int motorCANID = 7;
            }

            public static final class EndEffectorMap {
                public static final int motorCANID = 7;
            }
        }

        public static class ClimberMap {
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
