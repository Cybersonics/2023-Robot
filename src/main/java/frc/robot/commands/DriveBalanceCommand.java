// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.NavXGyro;

public class DriveBalanceCommand extends CommandBase {

    private Drive _drive;
    private NavXGyro _navXGyro;

    public static final double OMEGA_SCALE = 1.0 / 45.0;// 30
    public static final double DEADZONE_LSTICK = 0.07;
    private static final double DEADZONE_RSTICK = 0.07;

    private static final double BalanceD = 0;
    private static final double BalanceP = 0;
    private static final double BalanceI = 0;
    PIDController balanceContoller;
    private double originHeading = 0.0;
    private double originRoll = 0.0;
    private double leftPow = 1;
    private double rightPow = 1;

    /**
     * Creates a new DriveCommand using a standard set of joysticks as the driver
     * joysticks.
     */
    public DriveBalanceCommand(Drive drive, NavXGyro gyro) {
        this._drive = drive;
        this._navXGyro = gyro;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        originHeading = _navXGyro.getZeroAngle();
        originRoll = _navXGyro.getRollAngle();
        balanceContoller = new PIDController(BalanceP, BalanceI, BalanceD);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double directionforce = balanceContoller.calculate(originRoll, 0);

        double stickForward = -directionforce;
        double stickStrafe = 0.0;
        double stickOmega = 0.0;

        double strafe = Math.pow(Math.abs(stickStrafe), leftPow) * Math.signum(stickStrafe);
        double forward = Math.pow(Math.abs(stickForward), leftPow) * Math.signum(stickForward);
        double omega = Math.pow(Math.abs(stickOmega), rightPow) * Math.signum(stickOmega) * OMEGA_SCALE;

        /*
         * If the input from the joystick is less than a dead zone value then set the
         * joystick output to zero. This prevents the robot from drifting due to the
         * joysticks
         * not fully returning to the zero position.
         * Note: take care when setting the deadzone value. If the value is set to a
         * high value,
         * the robot will move aggresively when the stick goes past the deadzone value.
         */
        if (Math.abs(strafe) < DEADZONE_LSTICK)
            strafe = 0.0;
        if (Math.abs(forward) < DEADZONE_LSTICK)
            forward = 0.0;
        if (Math.abs(omega) < DEADZONE_RSTICK * OMEGA_SCALE)
            omega = 0.0;

        this._drive.processInput(forward, strafe, omega, false, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this._drive.processInput(0.0, 0.0, 0.0, true, false);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return balanceContoller.atSetpoint();
    }
}