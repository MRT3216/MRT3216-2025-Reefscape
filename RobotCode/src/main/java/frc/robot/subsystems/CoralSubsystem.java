package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.settings.Constants.PivotConstants;
import frc.robot.settings.RobotMap.ROBOT.PivotMap;

public class CoralSubsystem {
    private static CoralSubsystem instance;
    private final SparkFlex pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private ProfiledPIDController PIDController;
    private ArmFeedforward feedforward;
    private boolean enabled;

    public CoralSubsystem() {
        pivotMotor = new SparkFlex(PivotMap.pivotMotorCANID, MotorType.kBrushless);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(PivotConstants.kMotorCurrentLimit)
                .voltageCompensation(PivotConstants.kVoltageCompensation)
                .openLoopRampRate(PivotConstants.kPivotRampRate);

        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        EncoderConfig encoderConfig = new EncoderConfig();
        encoderConfig.positionConversionFactor(360);
        pivotConfig.apply(encoderConfig);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // pivotConfig.softLimit(0, 0);

        PIDController = new ProfiledPIDController(
                PivotConstants.kPivotKp, 
                PivotConstants.kPivotKi, 
                PivotConstants.kPivotKd,
                new TrapezoidProfile.Constraints(PivotConstants.kMaxPivotVelocity, PivotConstants.kMaxPivotAcceleration));
        
        feedforward = new ArmFeedforward(
                PivotConstants.kPivotkS, 
                PivotConstants.kPivotkG, 
                PivotConstants.kPivotkV,
                PivotConstants.kPivotkA);
    }


    public void periodic(){
        if(enabled){
            // TODO: add values
            double feedforwardOutput = 0;
            double pidOutput = PIDController.calculate(pivotEncoder.getPosition(), 0);
            pivotMotor.setVoltage(feedforwardOutput + pidOutput);
        }
    }
    public void enable(){
        enabled = true;
        PIDController.reset(0); // TODO: add value
    }

    public void disable(){
        enabled = false;
        pivotMotor.set(0);
    }

    public double getPivotDegrees(){
        return pivotEncoder.getPosition() * PivotConstants.kScaleFactor - PivotConstants.kOffsetDegrees;
    }
}