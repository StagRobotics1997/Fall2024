package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.RobotState;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kUSB1);
    private SwerveDrivePoseEstimator mPoseEstimator;
    private SwerveDriveKinematics mKinematics;
    private static Swerve instance;
    private boolean mAligning = false;
    private frc.robot.Vision vision;

    public Swerve() {
        /**
         * Trustworthiness of the internal model of how motors should be moving Measured
         * in expected
         * standard deviation (meters of position and degrees of rotation)
         */
        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        /**
         * Trustworthiness of the vision system Measured in expected standard deviation
         * (meters of
         * position and degrees of rotation)
         */
        Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
        // gyro.calibrate();
        gyro.reset();
        mKinematics = Constants.SwerveConstants.swerveKinematics;

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };
        ModuleConfig[] mModuleConfigs = new ModuleConfig[] {

        };
        mPoseEstimator = new SwerveDrivePoseEstimator(
                Constants.SwerveConstants.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new edu.wpi.first.math.geometry.Pose2d(new edu.wpi.first.math.geometry.Translation2d(0, 0),
                        edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0)),
                stateStdDevs,
                visionMeasurementStdDevs);

         // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPose, 
        this::resetPose, 
        this::getSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(
          Constants.SwerveConstants.translationConstants,
          Constants.SwerveConstants.rotationConstants
        ),
        new RobotConfig(40.0, 10.0, new ModuleConfig(0.1, 2.0,.5, DCMotor.getKrakenX60(4), 12, 4), .99),
        () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public ChassisSpeeds getSpeeds() {
        return mKinematics.toChassisSpeeds(getModuleStates());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // public Command driveToPose(Pose2d pose) {
    //     // Create the constraints to use while pathfinding
    //     PathConstraints constraints = new PathConstraints(
    //             3.0, 4.0,
    //             Math.PI, Units.degreesToRadians(720));

    //     // Since AutoBuilder is configured, we can use it to build pathfinding commands
    //     return AutoBuilder.pathfindToPose(
    //             pose,
    //             constraints,
    //             0.0, // Goal end velocity in meters/sec
    //             0.0 // Rotation delay distance in meters. This is how far the robot should travel
    //                 // before attempting to rotate.
    //     );
    // }

    // public Command pathToPoint(Pose2d destination) {
    //     double xDiff = destination.getX() - getPose().getX();
    //     double yDiff = destination.getY() - getPose().getY();
    //     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
    //             new Pose2d(getPose().getX() + (xDiff / 3), getPose().getY() + (yDiff / 3), destination.getRotation()),
    //             new Pose2d(getPose().getX() + ((xDiff / 3) * 2), getPose().getY() + ((yDiff / 3) * 2),
    //                     destination.getRotation()),
    //             destination);

    //     System.out
    //             .println(String.format("StartPose x:%s, y:%s, r:%s", getPose().getX(), getPose().getY(), getHeading()));
    //     System.out.println(String.format("Waypoint 1 x:%s, y:%s, r:%s", getPose().getX() + (xDiff / 3),
    //             getPose().getY() + (yDiff / 3), destination.getRotation()));
    //     System.out.println(String.format("Waypoint 2 x:%s, y:%s, r:%s", getPose().getX() + ((xDiff / 3) * 2),
    //             getPose().getY() + ((yDiff / 3) * 2),
    //             destination.getRotation()));
    //     System.out.println(String.format("Destintion x:%s, y:%s, r:%s", destination.getX(), destination.getY(),
    //             destination.getRotation()));
    //     // Create the path using the bezier points created above
    //     PathPlannerPath path = new PathPlannerPath(
    //             bezierPoints,
    //             new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a
    //                                                                      // differential drivetrain, the angular
    //                                                                      // constraints have no effect.
    //             new GoalEndState(0.0, destination.getRotation()) // Goal end state. You can set a holonomic rotation
    //                                                              // here. If using a differential drivetrain, the
    //                                                              // rotation will have no effect.
    //     );
    //     return new FollowPathHolonomic(
    //             path,
    //             this::getPose, // Robot pose supplier
    //             this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //             this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
    //                                              // Constants class
    //                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                     new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                     4.5, // Max module speed, in m/s
    //                     0.4, // Drive base radius in meters. Distance from robot center to furthest module.
    //                     new ReplanningConfig() // Default path replanning config. See the API for the options here
    //             ),
    //             () -> {
    //                 // Boolean supplier that controls when the path will be mirrored for the red
    //                 // alliance
    //                 // This will flip the path being followed to the red side of the field.
    //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                     return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //             },
    //             this // Reference to this subsystem to set requirements
    //     );
    // }

    /**
     * TODO tune PID gains for x, y, and rot under {@link SwerveConstants} <br>
     * <br>
     * A method to travel to a given position on the field.
     *
     * @param targetPose a {@link Pose2d} representing the pose to travel to.
     * @return a Command that continuously attempts to converge to targetPose until
     *         the controllers'
     *         thresholds have been reached.
     */
    public Command goToPointCommand(Pose2d targetPose) {
        PIDController xController = new PIDController(0.65, 0, 0);
        PIDController yController = new PIDController(0.45, 0, 0);
        PIDController thetaController = new PIDController(0.1, 0, 0);
        xController.setTolerance(.02);
        yController.setTolerance(.02);
        thetaController.setTolerance(.07);
        mAligning = true;

        return new FunctionalCommand(
                () -> System.out.println(
                        String.format(
                                "Traveling to x:%s, y:%s, z:%s",
                                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees())),
                () -> {
                    double sX = xController.calculate(getPose().getX(), targetPose.getX());
                    double sY = yController.calculate(getPose().getY(), targetPose.getY());
                    double sR = thetaController.calculate(
                            getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

                    drive(new Translation2d(sX, sY).times(Constants.SwerveConstants.maxSpeed / 2),
                            sR * Constants.SwerveConstants.maxAngularVelocity / 2,
                            true,
                            true);
                    // drive(ChassisSpeeds.fromFieldRelativeSpeeds(sX, sY, sR,
                    // getPose().getRotation()) true);
                    // drive(sX, sY, sR, true, true);
                },
                interrupted -> {
                    xController.close();
                    yController.close();
                    thetaController.close();
                    mAligning = false;
                },
                () -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint(),
                /*
                 * or getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1
                 * ?
                 */
                this);
    }

    /**
     * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        mPoseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        mPoseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return mSwerveOdometry.getPoseMeters();
        return mPoseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void zeroHeading() {
        mPoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void stopModules() {
        for (SwerveModule mod : mSwerveMods) {
            mod.stop();
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = mKinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.SwerveConstants.maxSpeed);

        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(targetStates[i], false);
        }
    }

    @Override
    public void periodic() {
        if (!mAligning && !RobotState.isAutonomous())
        // && LimelightHelpers.getTV("limelight"))
        {
            var visionEst = vision.getEstimatedGlobalPose();
            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = vision.getEstimationStdDevs();
                        addVisionMeasurement(
                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });

        } else {
            mPoseEstimator.update(getGyroYaw(), getModulePositions());
        }
        SmartDashboard.putString("Pose", String.format("X=%.2f  Y=%.2f  R=%.4f",
                mPoseEstimator.getEstimatedPosition().getX(),
                mPoseEstimator.getEstimatedPosition().getY(),
                mPoseEstimator.getEstimatedPosition().getRotation().getDegrees()));
        SmartDashboard.putNumber("Gyro ", getGyroYaw().getDegrees());
        for (SwerveModule mod : mSwerveMods) {
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}