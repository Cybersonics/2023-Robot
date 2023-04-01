package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.CounterBase;

public class Arm extends SubsystemBase {

    private static Arm instance;

    private CANSparkMax extensionMotor;
    
    private CANSparkMax shoulderMotor;
    private SparkMaxPIDController shoulderPIDController;
    
    private final Encoder arm_QuadEncoder;
    private final DutyCycleEncoder arm_AbsEncoder;

    private AnalogInput extPot;
    
    public static Arm getInstance() {
        if(instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        extPot = new AnalogInput(0);

        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ArmConstants.shoulderMotorId, MotorType.kBrushless);

        arm_QuadEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k4X);
        /*
        * Defines the number of samples to average when determining the rate.
        * On a quadrature encoder, values range from 1-255;
        * larger values result in smoother but potentially
        * less accurate rates than lower values.
        */
        arm_QuadEncoder.setSamplesToAverage(5);
        /*
        * Defines how far the mechanism attached to the encoder moves per pulse. In
        * this case, we assume that a 360 count encoder is directly
        * attached to a 3 inch diameter (1.5inch radius) wheel,
        * and that we want to measure distance in inches.
        */
        arm_QuadEncoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
        /*
        * Defines the lowest rate at which the encoder will
        * not be considered stopped, for the purposes of
        * the GetStopped() method. Units are in distance / second,
        * where distance refers to the units of distance
        * that you are using, in this case inches.
        */
        arm_QuadEncoder.setMinRate(1.0);


        // Initializes a duty cycle encoder on DIO pins 0
        arm_AbsEncoder = new DutyCycleEncoder(0);
        // Configures the encoder to return a distance of 4 for every rotation
        arm_AbsEncoder.setDistancePerRotation(360.0);
        
        shoulderPIDController = extensionMotor.getPIDController();
        shoulderPIDController.setP(0.0);
        shoulderPIDController.setI(0.0);
        shoulderPIDController.setD(0.0);


        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        extensionMotor.restoreFactoryDefaults();
        // shoulderMotor.restoreFactoryDefaults();
        
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        shoulderMotor.setInverted(true);
        shoulderMotor.setOpenLoopRampRate(.5);
        //shoulderMotor.setSmartCurrentLimit(ArmConstants.kshoulderMotorCurrentLimit);
        
        extensionMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setSmartCurrentLimit(ArmConstants.extensionMotorCurrentLimit);

        // extensionMotor.setInverted(true);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        extensionMotor.burnFlash();
        // shoulderMotor.burnFlash();
    }

    public void shoulderMove(double speed) {        
        SmartDashboard.putNumber("Current Arm Position", getAbsArmPos());
        shoulderMotor.set(speed);
    }

    public void extensionMove(double speed) {
        SmartDashboard.putNumber("Current Extension Position", getExtensionPosition());
        extensionMotor.set(speed);
    }

    public double getExtensionPosition() {
        return extPot.getValue();
    }

    public double getAbsArmPos() {
        return arm_AbsEncoder.getDistance();
    };

    public double getQuadPos() {
        return arm_QuadEncoder.getDistance();
    }

    public double get_QuadArmRate() {
        return arm_QuadEncoder.getRate();
    }

}
