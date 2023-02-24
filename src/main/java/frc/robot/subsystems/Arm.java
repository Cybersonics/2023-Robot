package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private static Arm instance;

    private CANSparkMax shoulderMotor;
    private CANSparkMax extensionMotor;

    private RelativeEncoder extensionEncoder;
    private AbsoluteEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPIDController;

    private Arm() {
        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ArmConstants.shoulderMotorId, MotorType.kBrushless);

        
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        extensionMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults();

        // Setup encoders and PID controllers for the shoulder and extension SPARKS MAX.
        extensionEncoder = extensionMotor.getEncoder();

        shoulderEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // shoulderPIDController = shoulderMotor.getPIDController();
        // shoulderPIDController.setFeedbackDevice(shoulderEncoder);

        // Apply position and velocity conversion factors for the shoulder encoder.
        // shoulderEncoder.setPositionConversionFactor(ArmConstants.shoulderEncoderPositionFactor);
        // shoulderEncoder.setVelocityConversionFactor(ArmConstants.shoulderEncoderVelocityFactor);

        // Set the PID gains for the shoulder motor. 
        // Note these are example gains, and you may need to tune them for your own robot!
        // shoulderPIDController.setP(ArmConstants.shoulderP);
        // shoulderPIDController.setI(ArmConstants.shoulderI);
        // shoulderPIDController.setD(ArmConstants.shoulderD);
        // shoulderPIDController.setOutputRange(ArmConstants.shoulderMinOutput, ArmConstants.shoulderMaxOutput);

        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotor.setInverted(true);
        shoulderMotor.setOpenLoopRampRate(3);
        //shoulderMotor.setSmartCurrentLimit(ArmConstants.kshoulderMotorCurrentLimit);
        
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setInverted(true);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        extensionMotor.burnFlash();
        shoulderMotor.burnFlash();
    }

    public void shoulderMove(double speed) {
        shoulderMotor.set(speed);
    }

    public void setShoulderPosition(double position) {
        shoulderPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public void extensionMove(double speed) {
        extensionMotor.set(speed);
    }

    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }
        return instance;
    }
}
