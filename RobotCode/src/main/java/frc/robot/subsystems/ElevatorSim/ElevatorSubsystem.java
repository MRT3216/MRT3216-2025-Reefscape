package frc.robot.subsystems.ElevatorSim;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.RobotMap.ROBOT.ElevatorMap;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex leadMotorController;
    private final SparkFlex followerMotorController;
    private SparkRelativeEncoder encoder;
    private ProfiledPIDController m_controller;
    private ElevatorSimulation m_elevatorSimContainer;
    private ElevatorFeedforward m_elevatorFeedForward;
    private boolean m_enabled;

    // TODO: replace with actual values
    public ElevatorSubsystem() {
        leadMotorController = new SparkFlex(ElevatorMap.leadMotorCANID, MotorType.kBrushless);
        followerMotorController = new SparkFlex(ElevatorMap.followerMotorCANID, MotorType.kBrushless);

        SparkMaxConfig leadConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();

        leadConfig.idleMode(IdleMode.kBrake)
                .inverted(ElevatorConstants.kLeftMotorInverted)
                .smartCurrentLimit(ElevatorConstants.kMotorCurrentLimit)
                .voltageCompensation(ElevatorConstants.kVoltageCompensation);

        // AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        // encoderConfig.positionConversionFactor(360);
        // leadConfig.apply(encoderConfig);

        leadMotorController.configure(leadConfig, null, PersistMode.kPersistParameters);

        followerConfig.follow(leadMotorController, false)
                .idleMode(IdleMode.kBrake)
                .inverted(ElevatorConstants.kRightMotorInverted)
                .smartCurrentLimit(ElevatorConstants.kMotorCurrentLimit)
                .voltageCompensation(ElevatorConstants.kVoltageCompensation);

        followerMotorController.configure(followerConfig, null, PersistMode.kPersistParameters); // TODO: improve this bs

        encoder = (SparkRelativeEncoder) leadMotorController.getEncoder();

        m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp, ElevatorConstants.kElevatorKi,
                ElevatorConstants.kElevatorKd, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxElevatorVelocity,
                        ElevatorConstants.kMaxElevatorAcceleration));

        m_elevatorFeedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS,
                ElevatorConstants.kElevatorkG, ElevatorConstants.kElevatorkV, ElevatorConstants.kElevatorkA);

        m_controller.setTolerance(ElevatorConstants.kPositionTolerance);
        m_controller.calculate(0);

        m_enabled = false;

        if (RobotBase.isSimulation()) {
            this.m_elevatorSimContainer = new ElevatorSimulation(encoder, leadMotorController);
        }
    }

    public void periodic() {
        if (m_enabled == false) {
            stop();
            return;
        }

        double pidOutput = m_controller.calculate(encoder.getPosition());
        double feedForward = m_elevatorFeedForward.calculate(m_controller.getSetpoint().velocity);
        double motorEffortVoltage = pidOutput + feedForward;

        SmartDashboard.putNumber("Motor voltage", motorEffortVoltage);
        SmartDashboard.putNumber("Encoder Position",         encoder.getPosition());

        leadMotorController.setVoltage(12);
    }

    protected void enable() {
        m_enabled = true;
    }

    protected void stop() {
        m_controller.setGoal(encoder.getPosition());
        leadMotorController.set(0);
        m_enabled = false;
    }

    protected void setGoal(double goal) {
        m_controller.setGoal(goal);
    }

    protected boolean atGoal() {
        return m_controller.atGoal();
    }

    @Override
    public void simulationPeriodic() {
        if (m_elevatorSimContainer != null) {
            m_elevatorSimContainer.simulationPeriodic();
            SmartDashboard.putBoolean("Elevator Enabled", m_enabled);
            SmartDashboard.putNumber("Elevator position error", m_controller.getPositionError());
            SmartDashboard.putNumber("Elevator position setpoint", m_controller.getSetpoint().position);
            SmartDashboard.putNumber("Elevator position actual", encoder.getPosition());
            SmartDashboard.putNumber("Motor effort", leadMotorController.get());
        }
    }
}