package frc.robot.commands;

import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class LiftRobot extends Command {    
    private LiftSubsystem mLiftSubsystem;    

    public LiftRobot(LiftSubsystem LiftSubsystem) {
        this.mLiftSubsystem = LiftSubsystem;
        addRequirements(LiftSubsystem);
    }
    
}