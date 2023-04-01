package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ArmPositionCommand extends CommandBase {

    private Arm _arm;
    private double _expectedShoulderPosition;
    private double _expectedExtensionPosition;
    private double _duration;
    private Timer _timer;

    private PIDController _shoulderPIDController;
    private final double shoulderP = 0.03;
    private final double shoulderI = 0.0;
    private final double shoulderD = 0.0;

    private PIDController _extensionPIDController;
    private final double extensionP = 0.001;
    private final double extensionI = 0.0;
    private final double extensionD = 0.0;

    /** Creates a new ArmCommand. */
    public ArmPositionCommand(Arm arm, double shoulderPosition, double extensionPosition, double runDurationInSeconds) {
        this._arm = arm;

        this._expectedShoulderPosition = shoulderPosition;
        this._expectedExtensionPosition = extensionPosition;

        this._duration = runDurationInSeconds;

        this._shoulderPIDController = new PIDController(shoulderP, shoulderI, shoulderD);
        this._shoulderPIDController.setTolerance(2);

        this._extensionPIDController = new PIDController(extensionP, extensionI, extensionD);
        this._extensionPIDController.setTolerance(5);

        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        _timer = new Timer();
        _timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (_expectedShoulderPosition > ArmConstants.shoulderEncoderBottom
                && _expectedShoulderPosition < ArmConstants.shoulderEncoderMax) {
            //System.out.println("going to shoulder position: " + _expectedShoulderPosition + " at " + _arm.getAbsArmPos());
            double calculatedChange = _shoulderPIDController.calculate(_arm.getAbsArmPos(), _expectedShoulderPosition);
            double speed = MathUtil.clamp(calculatedChange, -1, 1);

            SmartDashboard.putNumber("Calculated Shoulder Change", calculatedChange);

            _arm.shoulderMove(speed);
        }
        if (_shoulderPIDController.atSetpoint()) {
            if (_expectedExtensionPosition > ArmConstants.extensionEncoderIn
                    && _expectedExtensionPosition < ArmConstants.extensionEncoderOut) {
                System.out.println("going to extension position: " + _expectedExtensionPosition + " at "
                        + _arm.getExtensionPosition());
                double calculatedChange = _extensionPIDController.calculate(_arm.getExtensionPosition(),
                        _expectedExtensionPosition);
                double speed = MathUtil.clamp(calculatedChange, -1, 1);

                SmartDashboard.putNumber("Calculated Extension Change", calculatedChange);

                _arm.extensionMove(-speed);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _arm.shoulderMove(0);
        _arm.extensionMove(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (_shoulderPIDController.atSetpoint() && _extensionPIDController.atSetpoint())
                || _timer.hasElapsed(_duration);
    }
}