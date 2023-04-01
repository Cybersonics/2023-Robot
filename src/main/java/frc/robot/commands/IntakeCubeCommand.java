package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCubeCommand extends CommandBase {

    private Intake _intake;
    private Timer _timer;
    private boolean _isOut;
    private double _runDuration;

    /** Creates a new IntakeCommand. */
    public IntakeCubeCommand(Intake intake, boolean isOut, double runDurationInSeconds) {
        _intake = intake;
        _isOut = isOut;
        _runDuration = runDurationInSeconds;

        addRequirements(intake);
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
        if (_isOut) {
            // if (_intake.getOutputCurrent() < 15) {
                _intake.intakeMove(-.7);
            // }
        } else if (!_isOut) {
            // if (_intake.getOutputCurrent() < 15) {
                _intake.intakeMove(.7);
            // }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        _intake.intakeMove(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return _timer.hasElapsed(_runDuration);
    }
}