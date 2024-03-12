package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

public class Swerve extends SubsystemBase {

    public static double MAX_VOLTAGE = 12;
    public int DRIVE_MODE = 0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 20;
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

    StructArrayPublisher<SwerveModuleState> current_states = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructArrayTopic("Current Module States", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructArrayTopic("Target Module States", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Current pose", Pose2d.struct).publish();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(), new Pose2d(0,0, rotation()));
    private final KrakenSwerveModule[] modules = new KrakenSwerveModule[4];
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private boolean active = true;

    public Swerve() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++){
            modules[i] = new KrakenSwerveModule(
                tab.getLayout(DriveConstants.LAYOUT_TITLE[i], BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(i * 2, 0),
                DriveConstants.DRIVE_ID[i],
                DriveConstants.STEER_ID[i],
                DriveConstants.ENCODER_ID[i]);
        }

        AutoBuilder.configureHolonomic(
                this::pose,
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

    public double distance(Pose2d reference_point){
        Transform2d dist = odometry.getPoseMeters().minus(reference_point);
        return Math.sqrt( (dist.getX() * dist.getX()) + (dist.getY() * dist.getY()) );
    }

    private SwerveModulePosition modulePosition(KrakenSwerveModule module) {
        return new SwerveModulePosition(module.drivePosition(), Rotation2d.fromRadians(module.angle()));
    }

    private SwerveModuleState moduleState(KrakenSwerveModule module){
        return new SwerveModuleState(module.velocity(), new Rotation2d(module.angle()));
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < modules.length; i++)
            pos[i] = modulePosition(modules[i]);
        return pos;
    }
    
    public SwerveModuleState[] moduleStates(KrakenSwerveModule[] modules){
        SwerveModuleState[] state = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = moduleState(modules[i]);
        return state;
    }

    public Pose2d pose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(180);
        resetPosition();
        
        odometry.resetPosition(rotation(), modulePositions(), pose);
        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void setOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();
        
        odometry.resetPosition(rotation(), modulePositions(), pose);
        odometry.resetPosition(rotation(), modulePositions(), pose);
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
        current_states.set(moduleStates(modules));
        target_states.set(states);
        Pose2d pose = odometry.update(rotation(), modulePositions());
        posePublisher.set(pose);

        SmartDashboard.putNumber("X position", pose.getX());
        SmartDashboard.putNumber("Y position", pose.getY());

        SmartDashboard.putNumber("Odometry rotation", rotation().getDegrees());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon2.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Pitch", pigeon2.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Pigeon Roll", pigeon2.getRoll().getValueAsDouble());

        SmartDashboard.putString("Drive Mode", DriveConstants.DRIVE_MODE_DISPLAY[DRIVE_MODE]);
    }

    public static final class DriveConstants {
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(25.5);
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(27.5);

        public static final int[] DRIVE_ID = {4, 2, 6, 8}; // FL, FR, BL, BR
        public static final int[] STEER_ID = {5, 4, 7, 9}; // FL, FR, BL, BR
        public static final int[] ENCODER_ID = {11, 10, 12, 13}; // FL, FR, BL, BR
        public static final String[] LAYOUT_TITLE = {"Front Left", "Front Right", "Back Left", "Back Right"};

        public static final int PIGEON_ID = 14;

        // We are not dealing with enums being class BS
        public static final String[] DRIVE_MODE_DISPLAY = {"Robot-Oriented", "Field-Oriented", "Fixed-Point", "Fixed Alignment"};
        public static final int ROBOT_ORIENTED = 0;
        public static final int FIELD_ORIENTED = 1;
        public static final int FIXED_POINT_TRACKING = 2;
        public static final int FIXED_ALIGNMENT = 3;


    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 0.25;

        public static final double kPXController = 20;
        public static final double kPThetaController = 22;
    }
}