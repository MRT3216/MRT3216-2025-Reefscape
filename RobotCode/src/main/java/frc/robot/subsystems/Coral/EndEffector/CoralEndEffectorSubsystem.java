package frc.robot.subsystems.Coral.EndEffector;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.CORAL.END_EFFECTOR;
import frc.robot.settings.RobotMap.ROBOT.CORAL_SYSTEM.END_EFFECTOR_MAP;

public class CoralEndEffectorSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final LaserCan laserCan;

    public CoralEndEffectorSubsystem() {
        motor = new TalonFX(END_EFFECTOR_MAP.motorCANID);
        laserCan = new LaserCan(END_EFFECTOR_MAP.laserCanCANID);

        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
        motorConfigurator.apply(END_EFFECTOR.motorConfiguration);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Has Coral", hasCoral().getAsBoolean());
    }

    private boolean coralInIntake() {
        LaserCan.Measurement measurement = laserCan.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            if (measurement.distance_mm < 50) {
                return true;
            }
        }
        return false;
    }

    private void runAtSpeed(double speed) {
        motor.set(speed);
    }

    private void stopIntake() {
        motor.set(0);
    }

    // region Commands and Triggers

    public Command runEndEffectorCommand() {
        return this.defer(
                () -> {
                    if (coralInIntake()) {
                        return outtakeCoralCommand();
                    } else {
                        return intakeCoralCommand();
                    }
                });
    }

    public Command intakeCoralCommand() {
        return this.startEnd(
                () -> runAtSpeed(END_EFFECTOR.intakeSpeed),
                () -> stopIntake())
                .until(hasCoral());
    }

    public Command outtakeCoralCommand() {
        return this.run(
                () -> runAtSpeed(END_EFFECTOR.outtakeSpeed)).withTimeout(1)
                .andThen(() -> stopIntake());
    }

    public Trigger hasCoral() {
        return new Trigger(this::coralInIntake);
    }

    public Trigger noCoral() {
        return new Trigger(() -> !coralInIntake());
    }

    // endregion
}