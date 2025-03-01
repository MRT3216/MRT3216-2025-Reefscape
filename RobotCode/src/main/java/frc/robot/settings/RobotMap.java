package frc.robot.settings;

public final class RobotMap {
    public static final class ROBOT {
        public static final class CORAL_SYSTEM {
            public static final class ELEVATOR_MAP {
                public static final int leadMotorCANID = 1;
                public static final int followerMotorCANID = 2;
            }

            public static final class PIVOT_MAP {
                public static final int motorCANID = 6;
            }

            public static final class END_EFFECTOR_MAP {
                public static final int motorCANID = 7;
            }
        }

        public static final class ALGAE_SYSTEM {
            public static final class PIVOT_MAP {
                public static final int motorCANID = 4;
            }

            public static final class ROLLERS_MAP {
                public static final int motorCANID = 5;
            }
        }

        public static class CLIMBER_MAP {
            public static final int motorCANID = 3;
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
