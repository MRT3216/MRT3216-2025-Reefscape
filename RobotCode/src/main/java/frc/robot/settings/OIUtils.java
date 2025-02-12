package frc.robot.settings;

public class OIUtils {
	public static double modifyAxis(double value, double expo) {
		// Deadband
		//value = MathUtil.applyDeadband(value, OI.kJoystickDeadband);

		value = expo(value, expo);
		return value;
	}

	private static double expo(double value, double expo) {
		double adjValue = (1 - ((100 - expo) / 100)) * Math.pow(value, 3) + (value * ((100 - expo) / 100));

		return adjValue;
	}
}