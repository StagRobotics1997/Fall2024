package frc.robot.commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetLEDsForNote extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private LEDsubsystem mLEDsubsystem;

    public SetLEDsForNote(IntakeSubsystem intake, LEDsubsystem led) {
        this.mIntakeSubsystem = intake; 
        this.mLEDsubsystem = led;
        addRequirements(intake, led);
    }

    @Override
    public void execute() {
        System.out.println("exec");
        if (mIntakeSubsystem.noteDetected()) {
            System.out.println("green");
            mLEDsubsystem.setColor("green", 0, 131);

        } else {
            System.out.println("red");
            mLEDsubsystem.setColor("red", 0, 131);
        }
    }
}