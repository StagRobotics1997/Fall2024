package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAmp extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    Timer mTimer = new Timer();

    public ShootAmp(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.mShooterSubsystem = shooterSubsystem;
        this.mIntakeSubsystem = intakeSubsystem;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        mShooterSubsystem.startAmp();
        Timer.delay(.75);
        mIntakeSubsystem.startMotors(-.3);
        mTimer.start();
    }

    @Override
    public void execute() {
        Timer.delay(.785);
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.stopMotors(false);
        mIntakeSubsystem.stopMotors(false);
    }

    @Override
    public boolean isFinished() {
        if (mTimer.hasElapsed(1)) {
            return true;
        } else {
            return false;
        }
    }

}