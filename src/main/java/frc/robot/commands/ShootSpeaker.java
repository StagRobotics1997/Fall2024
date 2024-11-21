package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ShootSpeaker extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    Timer mTimer = new Timer();

    public ShootSpeaker(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.mShooterSubsystem = shooterSubsystem;
        this.mIntakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        //CommandScheduler.getInstance().cancelAll();
        mShooterSubsystem.startMotors();
        Timer.delay(.25);
        mIntakeSubsystem.startMotors(-.5);
        mTimer.start();
    }

    @Override
    public void execute() {
        Timer.delay(.25);
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.stopMotors(false);
        mIntakeSubsystem.stopMotors(false);
    }

    @Override
    public boolean isFinished() {
        if (mTimer.hasElapsed(0.25)) {
            return true;
        } else {
            return false;
        }
    }

}