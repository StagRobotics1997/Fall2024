package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowMode;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowMode) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowMode = slowMode;
    }

    @Override
    public void execute() {
        double factor = 1.0;
        //Baby Mode 
        // if (slowMode.getAsBoolean()) {
        //     factor = 0.25;
        // }
        /* Get desired movement Values, apply Deadband */
        double translationVal = MathUtil.applyDeadband(-translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(-strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if( alliance.get() == DriverStation.Alliance.Red){
                translationVal=translationVal*-1;
                strafeVal=strafeVal*-1;
            }
        }
        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationVal * factor, strafeVal * factor)
                        .times(Constants.SwerveConstants.maxSpeed),
                rotationVal * Constants.SwerveConstants.maxAngularVelocity * factor,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}