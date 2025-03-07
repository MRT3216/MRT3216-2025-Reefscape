package frc.robot.subsystems.Coral.EndEffector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

        // TODO: Need to finish this configuration
        TalonFXConfigurator motorConfigurator = motor.getConfigurator();
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = END_EFFECTOR.kMotorInverted;
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimit = END_EFFECTOR.kMotorCurrentLimit;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;

        motorConfigurator.apply(motorConfiguration);
        motorConfigurator.apply(motorOutputConfigs);
        motorConfigurator.apply(currentLimitsConfigs);
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

    public Command runEndEffector() {
        return this.defer(
                () -> {
                    if (coralInIntake()) {
                        return outtakeCoral();
                    } else {
                        return intakeCoral();
                    }
                });
    }

    private Command intakeCoral() {
        return this.startEnd(
                () -> runAtSpeed(END_EFFECTOR.intakeSpeed),
                () -> stopIntake())
                .until(hasCoral());
    }

    private Command outtakeCoral() {
        return this.startEnd(
                () -> runAtSpeed(END_EFFECTOR.outtakeSpeed),
                () -> stopIntake())
                .until(noCoral());
    }

    private Trigger hasCoral() {
        return new Trigger(this::coralInIntake);
    }

    private Trigger noCoral() {
        return new Trigger(() -> !coralInIntake());
    }

    // endregion
}