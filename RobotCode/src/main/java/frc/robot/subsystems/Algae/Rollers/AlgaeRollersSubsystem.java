package frc.robot.subsystems.Algae.Rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.settings.Constants.ALGAE.ROLLERS;
import frc.robot.settings.RobotMap.ROBOT.ALGAE_SYSTEM.ROLLERS_MAP;

public class AlgaeRollersSubsystem extends SubsystemBase {
    private final TalonFX motorController;
    private boolean isLastDirectionIntake = false;
    private AlgaeRollersSimulation rollerSimContainer;

    public AlgaeRollersSubsystem() {
        motorController = new TalonFX(ROLLERS_MAP.motorCANID);

        // TODO: Need to finish this configuration
        TalonFXConfigurator motorConfigurator = motorController.getConfigurator();
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfigurator.apply(motorConfiguration);
        if (RobotBase.isSimulation()) {
            //this.rollerSimContainer = new AlgaeRollersSimulation();
        }
    }

    private void setIntakeSpeed(double speed) {
        motorController.set(speed);
    }

    private void stopIntake() {
        motorController.set(0);
    }

    public Trigger hasAlgae() {
        // TODO: Finish this
        return new Trigger(() -> false);
    }

    // TODO: check this again
    public Command runRollerCommand() {
        if (isLastDirectionIntake) {
            isLastDirectionIntake = false;
            return this.startEnd(
                    () -> setIntakeSpeed(ROLLERS.outtakeSpeed),
                    () -> stopIntake());
        } else {
            isLastDirectionIntake = true;
            return this.startEnd(
                    () -> setIntakeSpeed(ROLLERS.intakeSpeed),
                    () -> stopIntake());
        }

        // // TODO: I think this needs more logic (check last years bot)
        // .until(() -> voltage > END_EFFECTOR.VOLTAGE_THRESHOLD);
    }

    public void periodic() {
        SmartDashboard.putNumber("Algae Roller Speed",
                motorController.getVelocity().getValueAsDouble());
    }
}