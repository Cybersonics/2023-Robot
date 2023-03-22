package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private static Intake instance;

    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);

        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(20);

        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        intakeMotor.burnFlash();
    }

    public void intakeMove(double speed) {
        intakeMotor.set(speed);
    }

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public double getOutputCurrent() {
        return intakeMotor.getOutputCurrent();
    }

}
