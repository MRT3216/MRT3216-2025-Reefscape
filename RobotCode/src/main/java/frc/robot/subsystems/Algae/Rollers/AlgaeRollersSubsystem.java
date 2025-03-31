package frc.robot.subsystems.Algae.Rollers;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private AlgaeRollersSimulation rollerSimContainer;

    public AlgaeRollersSubsystem() {
        motorController = new TalonFX(ROLLERS_MAP.motorCANID);

        TalonFXConfigurator motorConfigurator = motorController.getConfigurator();
        motorConfigurator.apply(ROLLERS.motorConfiguration);

        if (RobotBase.isSimulation()) {
            this.rollerSimContainer = new AlgaeRollersSimulation(motorController);
        }
    }

    private void setRollerSpeed(double speed) {
        motorController.set(speed);
    }

    private void stopRollers() {
        motorController.set(0);
    }

    public Command intakeAlgae() {
        return this.startEnd(
                () -> {
                    setRollerSpeed(ROLLERS.intakeSpeed);
                },
                () -> hold()).until(hasAlgaeTrigger());
    }

    public Command scoreAlgae() {
        return this.run(
                () -> setRollerSpeed(ROLLERS.outtakeSpeed)).withTimeout(1)
                .andThen(() -> stopRollers());
    }

    // From Team 3255: https://github.com/FRCTeam3255/2025_Robot_Code/blob/main/src/main/java/frc/robot/RobotContainer.java#L334
    private boolean hasAlgae() {
        Current intakeCurrent = motorController.getStatorCurrent().getValue();

        AngularVelocity intakeVelocity = motorController.getVelocity().getValue();
        double intakeAcceleration = motorController.getAcceleration().getValueAsDouble();

        Current intakeHasGamePieceCurrent = ROLLERS.HAS_ALGAE_CURRENT;
        AngularVelocity intakeHasGamePieceVelocity = ROLLERS.HAS_ALGAE_VELOCITY;

        if (intakeCurrent.gte(intakeHasGamePieceCurrent)
                && intakeVelocity.lte(intakeHasGamePieceVelocity)
                && intakeAcceleration < 0) {
            return true;
        } else {
            return false;
        }
    }

    private void hold() {
        VoltageOut voltageRequest = new VoltageOut(0);
        motorController.setControl(voltageRequest.withOutput(ROLLERS.HOLD_ALGAE_INTAKE_VOLTAGE));
    }

    public Trigger hasAlgaeTrigger() {
        return new Trigger(this::hasAlgae);
    }

    public void periodic() {
        SmartDashboard.putNumber("Algae Roller Speed", motorController.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Algae Roller Current", motorController.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        if (rollerSimContainer != null) {
            rollerSimContainer.simulationPeriodic();
        }
    }
}