package frc.robot.settings;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static final class DRIVETRAIN {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    public static final class VISION {

    }

    public static final class PATHING {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(7, 0, 0);
        // Create the constraints to use while pathfinding
        public static final PathConstraints pathConstraints = new PathConstraints(
                Constants.PATHING.maxVelocityMPS, Constants.PATHING.maxAccelerationMPSSq,
                Constants.PATHING.maxAngularVelocityRPS, Constants.PATHING.maxAngularAccelerationRPSS);
        public static final double maxVelocityMPS = 2;
        public static final double maxAccelerationMPSSq = 4;
        public static final double maxAngularVelocityRPS = Units.degreesToRadians(540);
        public static final double maxAngularAccelerationRPSS = Units.degreesToRadians(720);
        public static final double pathingMinimumDistance = 1;
        public static final double pathToCloseAlignEndVelocityMPS = 0;
        public static final double pathToCloseAlignEndVelocityReefMPS = 1;
    }

    public static final class CLOSE_PATHING {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(3.0, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(2.0, 0, 0);
        public static final double maxVelocityMPS = 1.0;
        public static final double maxAccelerationMPSSq = 2.0;
        public static final double maxAngularVelocityRPS = Units.degreesToRadians(180);
        public static final double maxAngularAccelerationRPS = Units.degreesToRadians(360);
    }

    public static final class FIELD_OFFSETS {
        public static double reefXOffsetCloseAdj = 0.56;
        public static double reefXOffsetInitial = 1.05;
        public static double reefYOffsetLeftBranch = Units.inchesToMeters(-6.469);
        public static double reefYOffsetRightBranch = -1 * reefYOffsetLeftBranch;
        public static double coralStationXOffset = 0.19;
        public static double processorXOffset = 0.5;

        public static Distance elevatorPrepCoralStationDistance = Meters.of(1);

        public static final Transform2d getReefOffsetPoseInitial(BranchSide side) {
            return new Transform2d(FIELD_OFFSETS.reefXOffsetInitial,
                    side.equals(BranchSide.LEFT) ? FIELD_OFFSETS.reefYOffsetLeftBranch
                            : FIELD_OFFSETS.reefYOffsetRightBranch,
                    Rotation2d.fromDegrees(180));
        }

        public static final Transform2d getReefOffsetPositionClose() {
            Transform2d transform = new Transform2d(
                    FIELD_OFFSETS.reefXOffsetCloseAdj,
                    0,
                    Rotation2d.fromDegrees(0));

            return transform;
        }

        public static final Transform2d coralStationOffsetPose = new Transform2d(
                FIELD_OFFSETS.coralStationXOffset, 0,
                Rotation2d.fromDegrees(180));

        public static final Transform2d processorOffset = new Transform2d(FIELD_OFFSETS.processorXOffset, 0,
                Rotation2d.fromDegrees(0));
    }

    public static final class OI {
        public static final double kJoystickDeadband = 0.1;
        public static final double kTranslationExpo = 0;
        public static final double kRotationExpo = 0;
    }

    public static final class CORAL {
        public enum POSITIONS {
            STOW(Meters.of(0.05), Degrees.of(20)),
            SCORE_PREP(Meters.of(0.3), Degrees.of(-60)),
            // CORAL_STATION(Meters.of(0.39), Degrees.of(19)),
            // TODO: Reset at comp field
            // CHECK THIS!!!!!
            CORAL_STATION(Meters.of(0.37), Degrees.of(21)),
            //CORAL_STATION(Meters.of(0.3), Degrees.of(22)),
            L1(Meters.of(0.05), Degrees.of(20)),
            L2(Meters.of(0.58), Degrees.of(-32)),
            L3(Meters.of(1.03), Degrees.of(-32)),
            L4(Meters.of(1.75), Degrees.of(-31));
            // L1(Meters.of(0), Degrees.of(20)),
            // L2(Meters.of(0.61), Degrees.of(-32)),
            // L3(Meters.of(1.02), Degrees.of(-32)),
            // L4(Meters.of(1.75), Degrees.of(-31));

            private Distance height;
            private Angle angle;

            private POSITIONS(Distance height, Angle angle) {
                this.height = height;
                this.angle = angle;
            }

            public Distance getHeight() {
                return height;
            }

            public Angle getAngle() {
                return angle;
            }
        }

        public static final class ELEVATOR {
            public static final double slowModeHeight = 0.9;
            public static final boolean kLeadMotorInverted = true;
            public static final int kMotorCurrentLimit = 60;
            public static final int kVoltageCompensation = 10;
            // // The soft limits are set in the motor controller to limit
            // // movement past a certain point. Consider this an emergency limit
            public static final Distance kSoftMaxHeight = Meters.of(1.77);;
            public static final Distance kSoftMinHeight = Meters.of(0);;
            // // These limits should be used to set how far we allow
            // // code to move the elevator. These should allow less movement than
            // // the soft limits
            public static final Distance kMaxHeight = Meters.of(1.75);
            public static final Distance kMinHeight = Inches.of(0.05);
            public static final Distance kPositionTolerance = Inches.of(1);

            public static final double kElevatorGearing = 60 / 7;
            public static final double kElevatorDrumRadius = Units.inchesToMeters(2.256 / 2);
            public static final Mass kCarriageMass = Kilograms.of(4.0); // kg

            public static final double kElevatorKp = 20;
            public static final double kElevatorKi = 0;
            public static final double kElevatorKd = 0;

            public static final double kElevatorkS = 0.1;// 0.02; // volts (V)
            public static final double kElevatorkG = 0.30; // volts (V)
            public static final double kElevatorkV = 3.3;//3.8; // volt per velocity (V/(m/s))
            public static final double kElevatorkA = 0;//0.17; // volt per acceleration (V/(m/s²))

            public static final LinearVelocity kMaxElevatorVelocity = Meters.of(2).per(Second); // m/s
            public static final LinearAcceleration kMaxElevatorAcceleration = Meters.of(3).per(Second).per(Second);
            public static final double kElevatorRampRate = 1;
        }

        public static final class PIVOT {
            public static final boolean kMotorInverted = false;
            public static final int kMotorCurrentLimit = 60;
            public static final int kVoltageCompensation = 10;
            public static final Angle kMaxPivotError = Degree.of(1.0); // Degrees

            public static final double kPivotGearing = (40 / 7) * (50 / 20);
            public static final Distance kPivotArmLength = Inches.of(12.9);
            public static final Mass kPivotMass = Pounds.of(2);

            // The soft limits are set in the motor controller to limit
            // movement past a certain point. Consider this an emergency limit
            // These are set to be with 0 as horiziontal
            public static final Angle kSoftReverseLimit = Degree.of(-85);
            public static final Angle kSoftForwardLimit = Degree.of(57);

            // These limits should be used to set how far we allow
            // code to move the arm. These should allow less movement than
            // the soft limits. Set to be with 0 as horiziontal
            public static final Angle kMinPivotAngle = Degree.of(-55);
            public static final Angle kMaxPivotAngle = Degree.of(52);
            //public static final Angle kStartingAngle = Degree.of(90);

            public static final double kMOI = SingleJointedArmSim.estimateMOI(kPivotArmLength.in(Meters),
                    kPivotMass.in(Kilograms));

            public static final double kPivotKp = 10;
            public static final double kPivotKi = 0;
            public static final double kPivotKd = 0;

            public static final double kPivotkS = 0; // volts (V)
            public static final double kPivotkG = 0.40;//0.3; // volts (V)
            public static final double kPivotkV = 2.39;//3; // volts * seconds / radians
            public static final double kPivotkA = 0.12; // volts * seconds^2 / radians

            public static final AngularVelocity kMaxAngularVelocity = DegreesPerSecond.of(360); // degrees per second
            public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecondPerSecond.of(450); // degrees per second squared max acceleration

            public static final double kPivotRampRate = 0.5;
        }

        public static final class END_EFFECTOR {
            public static final double intakeSpeed = 0.3;
            public static final double outtakeSpeed = -0.3;
            public static final double outtakeSpeedL1 = -0.25;

            public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            static {
                motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
                motorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 30;
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
                motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.5;
            }
        }
    }

    public static final class ALGAE {
        public static final class PIVOT {
            public enum Positions {
                INTAKING(Degrees.of(25)),
                STOW_SCORING(Degrees.of(90)),
                STARTING(Degrees.of(90));

                private Angle angle;

                private Positions(Angle angle) {
                    this.angle = angle;
                }

                public Angle getAngle() {
                    return angle;
                }
            }

            public static final boolean kMotorInverted = true;
            public static final int kMotorCurrentLimit = 60;
            public static final int kVoltageCompensation = 10;
            public static final Angle kMaxPivotError = Degree.of(1.0); // Degrees

            public static final double kPivotGearing = 72.0;
            public static final Distance kPivotArmLength = Inches.of(16.02);
            public static final Mass kPivotMass = Pounds.of(2);

            // The soft limits are set in the motor controller to limit
            // movement past a certain point. Consider this an emergency limit
            public static final Angle kSoftReverseLimit = Degree.of(0);
            public static final Angle kSoftForwardLimit = Degree.of(100);

            // These limits should be used to set how far we allow
            // code to move the arm. These should allow less movement than
            // the soft limits
            public static final Angle kMinPivotAngle = Degree.of(5);
            public static final Angle kMaxPivotAngle = Degree.of(90);

            public static final double kMOI = SingleJointedArmSim.estimateMOI(kPivotArmLength.in(Meters),
                    kPivotMass.in(Kilograms));

            public static final double kPivotKp = 50;
            public static final double kPivotKi = 0;
            public static final double kPivotKd = 0;

            public static final double kPivotkS = 0; // volts (V)
            public static final double kPivotkG = 0.1; // volts (V)
            public static final double kPivotkV = 3; // volts * seconds / radians
            public static final double kPivotkA = 0; // volts * seconds^2 / radians

            public static final AngularVelocity kMaxAngularVelocity = DegreesPerSecond.of(180); // degrees per second
            public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecondPerSecond.of(360); // degrees per second squared max acceleration

            public static final double kPivotRampRate = 0.5;
        }

        public static final class ROLLERS {
            public static final Current HAS_ALGAE_CURRENT = Amps.of(12);
            public static final double HOLD_ALGAE_INTAKE_VOLTAGE = 0.8;
            public static final AngularVelocity HAS_ALGAE_VELOCITY = RotationsPerSecond.of(75);
            public static final double intakeSpeed = 0.7;
            public static final double outtakeSpeed = -1.0;

            public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            static {
                motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
                motorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 30;
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
                motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.5;
            }
        }
    }

    public static final class CLIMBER {
        public static final boolean kMotorInverted = false;
        public static final int kMotorCurrentLimit = 80;
        public static final int kVoltageCompensation = 10;
        public static final double speed = 0.3;
    }

    public static final class SIMULATION {
        public static final double kSimulationTimeStep = 0.02; // seconds
        public static final double kVisualizationPixelMultiplier = 50;
    }

    public static final class AUTO {

    }

    public enum BargeCage {
        farCage, middleCage, closeCage
    }

    public enum CoralStationSide {
        LEFT,
        RIGHT
    }

    public enum BranchLevel {
        L4,
        L3,
        L2,
        L1
    }

    public enum ReefSide {
        ONE(0),
        TWO(5),
        THREE(4),
        FOUR(3),
        FIVE(2),
        SIX(1);

        private int value;

        private ReefSide(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    public enum BranchSide {
        LEFT,
        RIGHT
    }

    public enum ReefBranch {
        A(BranchSide.LEFT, ReefSide.ONE),
        B(BranchSide.RIGHT, ReefSide.ONE),
        C(BranchSide.LEFT, ReefSide.TWO),
        D(BranchSide.RIGHT, ReefSide.TWO),
        E(BranchSide.LEFT, ReefSide.THREE),
        F(BranchSide.RIGHT, ReefSide.THREE),
        G(BranchSide.LEFT, ReefSide.FOUR),
        H(BranchSide.RIGHT, ReefSide.FOUR),
        I(BranchSide.LEFT, ReefSide.FIVE),
        J(BranchSide.RIGHT, ReefSide.FIVE),
        K(BranchSide.LEFT, ReefSide.SIX),
        L(BranchSide.RIGHT, ReefSide.SIX);

        private BranchSide branchSide;
        private ReefSide reefSide;

        private ReefBranch(BranchSide branchSide, ReefSide reefSide) {
            this.branchSide = branchSide;
            this.reefSide = reefSide;
        }

        public BranchSide getBranchSide() {
            return branchSide;
        }

        public ReefSide getReefSide() {
            return reefSide;
        }
    }
}