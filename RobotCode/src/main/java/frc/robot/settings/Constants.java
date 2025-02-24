package frc.robot.settings;

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

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
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
		public static final double maxVelocityMPS = 4;
		public static final double maxAccelerationMPSSq = 6;
		public static final double maxAngularVelocityRPS = Units.degreesToRadians(540);
		public static final double maxAngularAccelerationRPSS = Units.degreesToRadians(720);
		public static final double pathingMinimumDistance = 1;
		public static final double pathToCloseAlignEndVelocityMPS = 3;
	}

	public static final class CLOSE_PATHING {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(3.0, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(2.0, 0, 0);
		public static final double maxVelocityMPS = 3.0;
		public static final double maxAccelerationMPSSq = 3.0;
		public static final double maxAngularVelocityRPS = Units.degreesToRadians(180);
		public static final double maxAngularAccelerationRPS = Units.degreesToRadians(180);
	}

	public static final class FIELD_OFFSETS {
		public static double reefXOffsetCloseAdj = 0.1;
		public static double reefXOffsetInitial = 0.6;
		public static double reefYOffsetLeftBranch = Units.inchesToMeters(-7);
		public static double reefYOffsetRightBranch = -1 * reefYOffsetLeftBranch;
		public static double coralStationXOffset = 0.5;
		public static double processorXOffset = 0.5;
		// TODO: This is offset additionally to compensate for the short field
		public static double cageFieldTooShortOffset = Units.inchesToMeters(-9.25);
		public static double cageFieldTooNarrowOffset = Units.inchesToMeters(-38.875);
		public static double cageXOffset = 0.0;
		public static double cageYOffset = 0.0;
		public static double cageRotation = 90;

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

		public static final Transform2d cageOffset = new Transform2d(
				FIELD_OFFSETS.cageXOffset + FIELD_OFFSETS.cageFieldTooShortOffset,
				FIELD_OFFSETS.cageYOffset + FIELD_OFFSETS.cageFieldTooNarrowOffset,
				Rotation2d.fromDegrees(FIELD_OFFSETS.cageRotation));
		public static final Transform2d processorOffset = new Transform2d(FIELD_OFFSETS.processorXOffset, 0,
				Rotation2d.fromDegrees(0));
	}

	public static final class OI {
		public static final double kJoystickDeadband = 0.1;
		public static final double kTranslationExpo = 0;
		public static final double kRotationExpo = 0;
	}

	public enum Positions {
		ZERO(Meters.of(0), CoralPivotConstants.kMinPivotAngle),
		STOW(Meters.of(0), Degrees.of(0)),
		L1(Meters.of(0.5), Degrees.of(0)),
		L2(Meters.of(1), Degrees.of(30)),
		L3(Meters.of(1.5), Degrees.of(45)),
		L4(Meters.of(2), Degrees.of(60));

		private Distance height;
		private Angle angle;

		private Positions(Distance height, Angle angle) {
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

	public static final class ElevatorConstants {
		public static final boolean kLeadMotorInverted = true;
		public static final int kMotorCurrentLimit = 60;
		public static final int kVoltageCompensation = 10;
		// public static final double kMaxPivotErrorDegrees = 1.0; // Degrees
		// // The soft limits are set in the motor controller to limit
		// // movement past a certain point. Consider this an emergency limit
		// public static final double kSoftForwardLimitDegrees = 105;
		// public static final double kSoftReverseLimitDegrees = 20;
		// // These limits should be used to set how far we allow
		// // code to move the arm. These should allow less movement than
		// // the soft limits
		// public static final double kMovementForwardLimitDegrees = 100;
		// public static final double kMovementReverseLimitDegrees = 15;
		// public static final double kOffsetDegrees = 0;
		// This is scaled by the gear ratio of encoder to pivot
		// public static final double kScaleFactor = 1.176;
		public static final Distance kPositionTolerance = Inches.of(0.5);

		public static final double kElevatorGearing = 10.0;
		public static final double kElevatorDrumRadius = Units.inchesToMeters(2);
		public static final Mass kCarriageMass = Kilograms.of(4.0); // kg

		public static final Distance kMinElevatorHeight = Meters.of(0);
		public static final Distance kMaxElevatorHeight = Meters.of(2);

		public static final double kElevatorKp = 5;
		public static final double kElevatorKi = 1;
		public static final double kElevatorKd = 0;

		public static final double kElevatorkS = 0.02; // volts (V)
		public static final double kElevatorkG = 0.9; // volts (V)
		public static final double kElevatorkV = 3.8; // volt per velocity (V/(m/s))
		public static final double kElevatorkA = 0.17; // volt per acceleration (V/(m/s²))

		public static final LinearVelocity kMaxElevatorVelocity = Meters.of(4).per(Second); // m/s
		public static final LinearAcceleration kMaxElevatorAcceleration = Meters.of(8).per(Second).per(Second);

		public static final double kElevatorRampRate = 0.1;
	}

	// create a class for the intake constants
	public static final class CoralIntakeConstants {
		public static final boolean kMotorInverted = false;
		public static final int kMotorCurrentLimit = 40;
		public static final int kVoltageCompensation = 10;
		public static final Angle kMaxError = Degree.of(1.0);

		public static final double VOLTAGE_THRESHOLD = 5; //TODO: Add correct value
	}

	public static final class CoralPivotConstants {
		public static final boolean kMotorInverted = false;
		public static final int kMotorCurrentLimit = 40;
		public static final int kVoltageCompensation = 10;
		public static final Angle kMaxPivotError = Degree.of(1.0); // Degrees

		public static final double kPivotGearing = 25.0;
		public static final Distance kPivotArmLength = Inches.of(12.9);
		public static final Mass kPivotMass = Pounds.of(5);

		public static final Angle kPivotOffset = Degrees.of(0);

		// The soft limits are set in the motor controller to limit
		// movement past a certain point. Consider this an emergency limit
		// TODO: Adjust these
		public static final Angle kSoftReverseLimit = Degree.of(-90);
		public static final Angle kSoftForwardLimit = Degree.of(90);

		// These limits should be used to set how far we allow
		// code to move the arm. These should allow less movement than
		// the soft limits
		public static final Angle kMinPivotAngle = Degree.of(-95);
		public static final Angle kMaxPivotAngle = Degree.of(95);

		public static final double kMOI = SingleJointedArmSim.estimateMOI(kPivotArmLength.in(Meters),
				kPivotMass.in(Kilograms));

		public static final double kPivotKp = 50;
		public static final double kPivotKi = 0;
		public static final double kPivotKd = 0;

		public static final double kPivotkS = 0;//0.0; // volts (V)
		public static final double kPivotkG = 0.7; // volts (V)
		public static final double kPivotkV = 0;//1.58; // volt per velocity (V/(m/s))
		public static final double kPivotkA = 0;//0.17; // volt per acceleration (V/(m/s²))

		public static final AngularVelocity kMaxAngularVelocity = DegreesPerSecond.of(180); // degrees per second
		public static final AngularAcceleration kMaxAngularAcceleration = DegreesPerSecondPerSecond.of(180); // degrees per second squared max acceleration

		public static final double kPivotRampRate = 0.1;
	}

	public static final class SimulationConstants {
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