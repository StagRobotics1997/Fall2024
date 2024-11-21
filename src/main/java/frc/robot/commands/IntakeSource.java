package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class IntakeSource extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private ShooterSubsystem mShooterSubsystem;
    Timer mTimer = new Timer();
    Boolean mComplete = false;

    public IntakeSource(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.mIntakeSubsystem = intakeSubsystem;
        this.mShooterSubsystem = shooterSubsystem;
        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
       // CommandScheduler.getInstance().cancelAll();
        mShooterSubsystem.startMotorsReverse();
        mTimer.start();
        mTimer.restart();
        mComplete = false;
    }

    @Override
    public void execute() {
        if (mIntakeSubsystem.noteDetected()) {
            mIntakeSubsystem.startMotorsReverse();
            Timer.delay(0.2);
            mIntakeSubsystem.stopMotors(false);
            mComplete = true;
        };
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.stopMotors(false);
        mIntakeSubsystem.stopMotors(false);
    }

    @Override
    public boolean isFinished() {
        if (mTimer.hasElapsed(IntakeConstants.timeOutSeconds)) {
            System.out.println("timeout waiting to detect note");
            return true;
        } else {
            return mComplete;
        }        
    }
}