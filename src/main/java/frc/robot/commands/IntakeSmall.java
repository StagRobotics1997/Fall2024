package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class IntakeSmall extends Command {
    private IntakeSubsystem mIntakeSubsystem;
        Timer mTimer = new Timer();
    Boolean mComplete = false;

    public IntakeSmall(IntakeSubsystem intakeSubsystem) {
        this.mIntakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
       // CommandScheduler.getInstance().cancelAll();
        // mShooterSubsystem.startMotorsReverse();
        mTimer.start();
        mTimer.restart();
        mComplete = false;
    }

    @Override
    public void execute() {
            mIntakeSubsystem.startMotorsReverse();
            Timer.delay(0.2);
            mIntakeSubsystem.stopMotors(false);
            mComplete = true;
    
    }

    @Override
    public void end(boolean interrupted) {
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