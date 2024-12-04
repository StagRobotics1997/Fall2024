package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.filter.SlewRateLimiter;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    // private final Joystick mDriveController = new Joystick(0);
    private final CommandJoystick m_primaryJoystick = new CommandJoystick(Constants.OIConstants.primaryJoystickChannel);
    private final CommandJoystick m_secondaryJoystick = new CommandJoystick(
            Constants.OIConstants.secondaryJoystickChannel);
    private final CommandJoystick mButtonBoard = new CommandJoystick(Constants.OIConstants.buttonBoardChannel);
    private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
    private final LiftSubsystem mLiftSubsystem = new LiftSubsystem();
    private final FlapSubsystem mFlapSubsystem = new FlapSubsystem();
    private PhotonCamera mCamera = new PhotonCamera("1");
    private Vision vision = new Vision();
    // private LEDsubsystem m_LEDsubsystem;

    private final SendableChooser<Command> autoChooser;
    /* Drive Controls */
    private final int translationAxis = (int) (-m_primaryJoystick.getRawAxis(1) * 100);
    private final int strafeAxis = (int) (m_primaryJoystick.getRawAxis(0) * 100);
    private final int rotationAxis = (int) (m_secondaryJoystick.getRawAxis(0) * 100);
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final SlewRateLimiter strafeFilter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter traslationFilter = new SlewRateLimiter(2.5);
    private final SlewRateLimiter rotationFilter = new SlewRateLimiter(2.5);
    /* Driver Buttons */
    // private final JoystickButton zeroGyro = new JoystickButton(mDriveController,
    // 1);
    // private final JoystickButton robotCentric = new
    // JoystickButton(mDriveController,2);
    // private final JoystickButton slowMode = new JoystickButton(m_primaryJoystick,
    // 1);
    // private final CommandJoystick m_primaryJoystick = new CommandJoystick(
    // frc.robot.Constants.ControllerConstants.PRIMARY_JOYSTICK);

    /* Subsystems */
    private final Swerve mSwerve = new Swerve();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // m_LEDsubsystem = new LEDsubsystem();

        // CommandScheduler.getInstance().setDefaultCommand(m_LEDsubsystem, new SetLEDsForNote(mIntakeSubsystem, m_LEDsubsystem));
        NamedCommands.registerCommand("IntakeGround", new IntakeGround(mIntakeSubsystem));
        NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker(mShooterSubsystem, mIntakeSubsystem));
        
        mSwerve.setVision(vision);
        mSwerve.setDefaultCommand(
                new TeleopSwerve(
                        mSwerve,
                        () -> traslationFilter.calculate(m_primaryJoystick.getRawAxis(1)),
                        () -> strafeFilter.calculate(m_primaryJoystick.getRawAxis(0)),
                        () -> rotationFilter.calculate(-m_secondaryJoystick.getRawAxis(0)),
                        () -> m_primaryJoystick.button(1).getAsBoolean(),
                        () -> m_secondaryJoystick.button(1).getAsBoolean()));
        // () -> robotCentric.getAsBoolean(),
       // () -> slowMode.getAsBoolean()));

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> mSwerve.zeroHeading(), mSwerve));
        m_primaryJoystick.button(6).onTrue(new InstantCommand(() -> mSwerve.zeroHeading(), mSwerve));
        m_secondaryJoystick.button(6).onTrue(mFlapSubsystem.InCommand());
        m_primaryJoystick.button(7).onTrue(Commands.runOnce(SignalLogger::start));
        m_secondaryJoystick.button(7).onTrue(Commands.runOnce(SignalLogger::stop));

/*
 * Joystick Y = quasistatic forward
 * Joystick A = quasistatic reverse
 * Joystick B = dynamic forward
 * Joystick X = dyanmic reverse
 */
// m_primaryJoystick.button(8).whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
// m_secondaryJoystick.button(8).whileTrue(m_mechanism.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
// m_primaryJoystick.button(9).whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kForward));
// m_secondaryJoystick.button(9).whileTrue(m_mechanism.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // Button Board Buttons
        mButtonBoard.button(1).onTrue(new IntakeGround(mIntakeSubsystem));
        mButtonBoard.button(3).onTrue(new ShootSpeaker(mShooterSubsystem, mIntakeSubsystem));
        // mButtonBoard.button(3).onTrue(new Angle(mAngleSubsystem));
        mButtonBoard.button(2).onTrue(new IntakeSource(mIntakeSubsystem, mShooterSubsystem));
        mButtonBoard.button(6).whileTrue(new InstantCommand(() -> mLiftSubsystem.startMotors(), mLiftSubsystem))
                .onFalse(new InstantCommand(() -> mLiftSubsystem.stopMotors(false), mLiftSubsystem));
        // mButtonBoard.button(5).onTrue(mFlapSubsystem.OutCommand());
        // mButtonBoard.button(5).onTrue(new AlignCommand(mSwerve));
// 
        mButtonBoard.button(4).onTrue(new ShootAmp(mShooterSubsystem, mIntakeSubsystem));
        mButtonBoard.button(7).onTrue(new HalfCourt( mShooterSubsystem, mIntakeSubsystem));
        mButtonBoard.button(8).onTrue(new InstantCommand(() ->CommandScheduler.getInstance().cancelAll()));
    }
    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 1
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
