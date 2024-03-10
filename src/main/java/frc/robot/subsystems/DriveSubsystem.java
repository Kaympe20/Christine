package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.modules.KrakenSwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

    public static double MAX_VOLTAGE = 12;
    public int DRIVE_MODE = 0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 20; //TODO: Increase the spead
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2,
                    DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2));

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public final Pigeon2 pigeon2 = new Pigeon2(DriveConstants.PIGEON_ID);

    StructArrayPublisher<SwerveModuleState> current_states = NetworkTableInstance.getDefault().getTable("Simulating")
            .getStructArrayTopic("myStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = NetworkTableInstance.getDefault().getTable("Simulating")
            .getStructArrayTopic("myStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Simulating")
            .getStructTopic("Pose2d", Pose2d.struct).publish();

    private final SwerveDriveOdometry odometry;
    private final KrakenSwerveModule[] modules = new KrakenSwerveModule[4];
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private boolean active = true;

    private final int tabWidth = 2;
    private final int tabHeight = 4;

    public DriveSubsystem() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        modules[0] = new KrakenSwerveModule(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(tabWidth, tabHeight)
                        .withPosition(0, 0),
                DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
                DriveConstants.FRONT_LEFT_TURN_MOTOR,
                DriveConstants.FRONT_LEFT_ENCODER);
        modules[1] = new KrakenSwerveModule(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(tabWidth, tabHeight)
                        .withPosition(2, 0),
                DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
                DriveConstants.FRONT_RIGHT_TURN_MOTOR,
                DriveConstants.FRONT_RIGHT_ENCODER);
        modules[2] = new KrakenSwerveModule(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList) // THIS IS A LONG CODE
                        .withSize(tabWidth, tabHeight)
                        .withPosition(4, 0),
                DriveConstants.BACK_LEFT_DRIVE_MOTOR,
                DriveConstants.BACK_LEFT_TURN_MOTOR,
                DriveConstants.BACK_LEFT_ENCODER);
        modules[3] = new KrakenSwerveModule(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(tabWidth, tabHeight)
                        .withPosition(6, 0),
                DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
                DriveConstants.BACK_RIGHT_TURN_MOTOR,
                DriveConstants.BACK_RIGHT_ENCODER);

        odometry = new SwerveDriveOdometry(kinematics, rotation(), getModulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::drive,
                new HolonomicPathFollowerConfig(
                        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0),
                        new PIDConstants(AutoConstants.kPThetaController, 0, 0.01),
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2,
                        new ReplanningConfig()),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return (alliance.isPresent()) && (DriverStation.Alliance.Red == alliance.get());
                },
                this);

    }

    public void zeroGyro() {
        pigeon2.setYaw(0);
    }

    public Rotation2d rotation() {
        return new Rotation2d(absoluteRotation());
    }

    public double absoluteRotation() {
        return Math.toRadians(pigeon2.getYaw().getValueAsDouble() % 360);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void stop() {
        chassisSpeeds = new ChassisSpeeds();
    }

    public double distance(double[] other_pose){
        Pose2d pose = odometry.getPoseMeters();
        double x = (pose.getX() - other_pose[0]);
        double y = (pose.getY() - other_pose[2]);
        return Math.sqrt( (x * x) + (y * y) );
    }

    private SwerveModulePosition getModulePosition(KrakenSwerveModule module) {
        return new SwerveModulePosition(module.drivePosition(), Rotation2d.fromRadians(module.steerAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++)
            pos[i] = getModulePosition(modules[i]);
        return pos;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(180);
        resetPosition();
        
        odometry.resetPosition(rotation(), getModulePositions(), pose);
        odometry.resetPosition(rotation(), getModulePositions(), pose);
    }

    public void setOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();
        
        odometry.resetPosition(rotation(), getModulePositions(), pose);
        odometry.resetPosition(rotation(), getModulePositions(), pose);
    }

    public void resetPosition() {
        for (KrakenSwerveModule mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (KrakenSwerveModule mod : modules)
            mod.resetSteerPosition();
    }

    public void resetAbsolute() {
        for (KrakenSwerveModule mod : modules)
            mod.resetAbsolute();
    }

    public void resetSteerPositions() {
        for (KrakenSwerveModule mod : modules)
            mod.set(0, 0);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        for (int i = 0; i < modules.length; i++)
            modules[i].set((states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
                    states[i].angle.getRadians());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public void activeChassis() {
        active = true;
    }

    public void disableChassis() {
        active = false;

        for (KrakenSwerveModule mod : modules)
            mod.stop();
    }

    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        if (active)
            setModuleStates(states);
        current_states.set(states);
        Pose2d pose = odometry.update(rotation(), getModulePositions());
        posePublisher.set(pose);

        // TODO: Wrap This Into A List, auto-order it too
        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getY());

        SmartDashboard.putNumber("Odometry rotation", rotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon2.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Pitch", pigeon2.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll", pigeon2.getRoll().getValueAsDouble());

        String DRIVE_MODE_DISPLAY = "";
        switch (DRIVE_MODE) {
            case DriveConstants.ROBOT_ORIENTED:
                DRIVE_MODE_DISPLAY = "Robot-Oriented";
                break;

            case DriveConstants.FIELD_ORIENTED:
                DRIVE_MODE_DISPLAY = "Field-Oriented";
                break;

            case DriveConstants.FIXED_POINT_TRACKING:
                DRIVE_MODE_DISPLAY = "Fixed-Point Tracking";
                break;

            case DriveConstants.FIXED_ALIGNMENT:
                DRIVE_MODE_DISPLAY = "Fixed Alignment";
                break;
        }

        SmartDashboard.putString("Drive Mode", DRIVE_MODE_DISPLAY);
    }

    public static final class DriveConstants {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25.5);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(27.5);

        public static final int FRONT_LEFT_DRIVE_MOTOR = 4;
        public static final int FRONT_LEFT_TURN_MOTOR = 5;
        public static final int FRONT_LEFT_ENCODER = 11;

        public static final int FRONT_RIGHT_DRIVE_MOTOR = 2;
        public static final int FRONT_RIGHT_TURN_MOTOR = 3;
        public static final int FRONT_RIGHT_ENCODER = 10;

        public static final int BACK_LEFT_DRIVE_MOTOR = 6;
        public static final int BACK_LEFT_TURN_MOTOR = 7;
        public static final int BACK_LEFT_ENCODER = 12;

        public static final int BACK_RIGHT_DRIVE_MOTOR = 8;
        public static final int BACK_RIGHT_TURN_MOTOR = 9;
        public static final int BACK_RIGHT_ENCODER = 13;

        public static final int PIGEON_ID = 14;

        // We are not dealing with enums being class BS
        public static final int ROBOT_ORIENTED = 0;
        public static final int FIELD_ORIENTED = 1;
        public static final int FIXED_POINT_TRACKING = 2;
        public static final int FIXED_ALIGNMENT = 3;

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.25;
        //public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kPXController = 20;
        public static final double kPThetaController = 22;
    }
}