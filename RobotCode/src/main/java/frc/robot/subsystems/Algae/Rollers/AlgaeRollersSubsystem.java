package frc.robot.subsystems.Algae.Rollers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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

        TalonFXConfigurator motorConfigurator = motorController.getConfigurator();
        TalonFXConfiguration motorConfiguration = ROLLERS.motorConfiguration;
        motorConfigurator.apply(motorConfiguration);

        if (RobotBase.isSimulation()) {
            this.rollerSimContainer = new AlgaeRollersSimulation(motorController);
        }
    }

    private void setIntakeSpeed(double speed) {
        motorController.set(speed);
    }

    private void stopIntake() {
        motorController.set(0);
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
    }

    public Command runRollers(double speed) {
        return this.startEnd(
                () -> setIntakeSpeed(speed),
                () -> stopIntake());
    }

    // From Team 3255: https://github.com/FRCTeam3255/2025_Robot_Code/blob/main/src/main/java/frc/robot/RobotContainer.java#L334
    private boolean hasAlgae() {
        Current intakeCurrent = motorController.getStatorCurrent().getValue();

        AngularVelocity intakeVelocity = motorController.getVelocity().getValue();
        double intakeAcceleration = motorController.getAcceleration().getValueAsDouble();

        Current intakeHasGamePieceCurrent = ROLLERS.HAS_ALGAE_CURRENT;
        AngularVelocity intakeHasGamePieceVelocity = ROLLERS.HAS_ALGAE_VELOCITY;

        if ((intakeCurrent.gte(intakeHasGamePieceCurrent))
                && (intakeVelocity.lte(intakeHasGamePieceVelocity))
                && (intakeAcceleration < 0)) {
            return true;
        } else {
            return false;
        }
    }

    public Trigger hasAlgaeTrigger() {
        return new Trigger(this::hasAlgae);
    }

    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        if (rollerSimContainer != null) {
            rollerSimContainer.simulationPeriodic();
            SmartDashboard.putNumber("Algae Roller Speed", motorController.getVelocity().getValueAsDouble());
        }
    }
}