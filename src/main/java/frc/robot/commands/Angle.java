package frc.robot.commands;

import frc.robot.subsystems.AngleSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class Angle extends Command {    
    private AngleSubsystem mAngleSubsystem;    

    public Angle(AngleSubsystem AngleSubsystem) {
        this.mAngleSubsystem = AngleSubsystem;
        addRequirements(AngleSubsystem);
    }

    @Override
    public void execute() {
        mAngleSubsystem.startMotors();
        Timer.delay(2);
        mAngleSubsystem.stopMotors(false);
    }
}