package frc.robot.commands;

import frc.robot.Constants.FlapConstants;
import frc.robot.subsystems.FlapSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class FlapOut extends Command {    
    private FlapSubsystem mFlapSubsystem;    

    public FlapOut(FlapSubsystem FlapSubsystem) {
        this.mFlapSubsystem = FlapSubsystem;
        addRequirements(FlapSubsystem);
    }
}