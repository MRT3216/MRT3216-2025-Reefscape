package frc.robot.settings;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public final class Constants {
	public static boolean DEBUG = false;

	public static final class DRIVETRAIN {
		public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
																									// desired top speed
		public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
																										// rotation per
																										// second max
																										// angular
																										// velocity

		public static final PIDConstants CLOSE_TRANSLATION_PID = new PIDConstants(3.0, 0, 0);
		public static final PIDConstants CLOSE_ROTATION_PID = new PIDConstants(2.0, 0, 0);
	}

	public static final class VISION {

	}

	public static final class PATHING {
		public static double maxVelocityMPS = 3.0;
		public static double maxAccelerationMPSSq = 3.0;
		public static double maxAngularVelocityRPS = Units.degreesToRadians(540);
		public static double maxAngularAccelerationRPSS = Units.degreesToRadians(720);
	}

	public static final class OI {
		public static final double kJoystickDeadband = 0.1;
		public static final double kTranslationExpo = 75;
		public static final double kRotationExpo = 0;
	}

	public static final class AUTO {
		public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
		public static final PIDConstants ANGLE_PID = new PIDConstants(7, 0, 0);
	}

	public static final class ELEVATOR {
		// TODO: add elevator constants
	}
}