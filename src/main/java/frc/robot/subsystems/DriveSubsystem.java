package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.modules.KrakenSwerveModule;
import frc.robot.modules.SwerveModule;
import frc.robot.utility.Constants.AutoConstants;
import frc.robot.utility.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveSubsystem extends SubsystemBase {
    
    public static double MAX_VOLTAGE = 12;
    public int drive_mode = 0;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3;
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
    StructArrayPublisher<SwerveModuleState> current_states = NetworkTableInstance.getDefault().getTable("Simulating").getStructArrayTopic("myStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> target_states = NetworkTableInstance.getDefault().getTable("Simulating").getStructArrayTopic("myStates", SwerveModuleState.struct).publish();
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault().getTable("Simulating").getStructTopic("Pose2d", Pose2d.struct).publish();


    private final SwerveDriveOdometry odometry;
    private final SwerveModule[] modules = {null, null, null, null};

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    private boolean slowMode = false;

    private boolean active = true;

    public DriveSubsystem() {
        DriveConstants.setOffsets();
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        modules[0] = new KrakenSwerveModule(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                DriveConstants.FRONT_LEFT_DRIVE_MOTOR,
                DriveConstants.FRONT_LEFT_TURN_MOTOR,
                DriveConstants.FRONT_LEFT_ENCODER,
                DriveConstants.FRONT_LEFT_ENCODER_OFFSET);
                modules[1] = new KrakenSwerveModule(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                DriveConstants.FRONT_RIGHT_DRIVE_MOTOR,
                DriveConstants.FRONT_RIGHT_TURN_MOTOR,
                DriveConstants.FRONT_RIGHT_ENCODER,
                DriveConstants.FRONT_RIGHT_ENCODER_OFFSET);
                modules[2] = new KrakenSwerveModule(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList) //THIS IS A LONG CODE
                        .withSize(2, 4)
                        .withPosition(4, 0),
                DriveConstants.BACK_LEFT_DRIVE_MOTOR,
                DriveConstants.BACK_LEFT_TURN_MOTOR,
                DriveConstants.BACK_LEFT_ENCODER,
                DriveConstants.BACK_LEFT_ENCODER_OFFSET);
                modules[3] = new KrakenSwerveModule(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                DriveConstants.BACK_RIGHT_DRIVE_MOTOR,
                DriveConstants.BACK_RIGHT_TURN_MOTOR,
                DriveConstants.BACK_RIGHT_ENCODER,
                DriveConstants.BACK_RIGHT_ENCODER_OFFSET);

        odometry = new SwerveDriveOdometry(kinematics, rotation(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::drive,
            new HolonomicPathFollowerConfig(
                new PIDConstants(AutoConstants.kPXController, 0, 0.01),
                new PIDConstants(AutoConstants.kPThetaController, 0, 0.01),
                AutoConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS/2,
                new ReplanningConfig()
            ),
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
        // double rot = Math.abs(pigeon2.getYaw()) % 360.0 * ((pigeon2.getYaw() < 0.0) ? -1.0 : 1.0);
        // return (rot < 0.0) ? rot + 360.0 : rot; 
        return Math.toRadians(pigeon2.getYaw().getValueAsDouble() %360);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public void stop(){ chassisSpeeds = new ChassisSpeeds();}

    private SwerveModulePosition getModulePosition(SwerveModule module) {
        return new SwerveModulePosition(module.drivePosition(), Rotation2d.fromRadians(module.steerAngle()));
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] pos = {null, null, null, null};
        for (int i = 0; i < modules.length; i++)
            pos[i] = getModulePosition(modules[i]);
        
        SmartDashboard.putNumber("FL Distance", pos[0].distanceMeters);
        SmartDashboard.putNumber("FR Distance", pos[1].distanceMeters);
        SmartDashboard.putNumber("BL Distance", pos[2].distanceMeters);
        SmartDashboard.putNumber("BR Distance", pos[3].distanceMeters);
        return pos;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d pose) {
        setOdometry(pose);
        setOdometry(pose);
    }

    public void setOdometry(Pose2d pose) {
        zeroGyro();
        resetPosition();
        odometry.resetPosition(rotation(), getModulePositions(), pose);
        odometry.resetPosition(rotation(), getModulePositions(), pose);
    }

    public void resetPosition() {
        for (SwerveModule mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (SwerveModule mod : modules)
            mod.resetSteerPosition();
    }

    public void resetAbsolute(){
        for (SwerveModule mod : modules)
            mod.resetAbsolute();
    }
    
    public void resetSteerPositions() {
        for (SwerveModule mod : modules)
            mod.set(0,0);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        
        // for (SwerveModule mod : modules)
        for (int i = 0; i < modules.length; i++)
            modules[i].set((states[i].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND) * MAX_VOLTAGE,
            states[i].angle.getRadians());
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    public void activeChassis(){
        active = true;
    }

    public void disableChassis(){
        active = false;

        for (SwerveModule mod : modules)
            mod.stop();
    }
    
    public void periodic() {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        if (active) setModuleStates(states);
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

        String drive_mode_display = "";
        switch(drive_mode){
            case 0: drive_mode_display = "Robot-Oriented";
            break;
            
            case 1: drive_mode_display = "Field-Oriented";
            break;
            
            case 2: drive_mode_display = "Fixed-Point Tracking";
            break;

            case 3: drive_mode_display = "Fixed Alignment";
            break;
        }

        SmartDashboard.putString("Drive Mode", drive_mode_display);
        SmartDashboard.putString("Drive Speed", slowMode ? "Slow" : "Normal");
    }
}
