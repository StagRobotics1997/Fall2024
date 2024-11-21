package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeGround extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    Timer mTimer = new Timer();

    public IntakeGround(IntakeSubsystem intakeSubsystem) {
        this.mIntakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        //CommandScheduler.getInstance().cancelAll();
        System.out.println("Starting Intake Ground");
        mIntakeSubsystem.startMotors(-.3);
        mTimer.start();
        mTimer.restart();
    }

    @Override
    public void execute() {
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
            return mIntakeSubsystem.noteDetected();
        }
    }
}