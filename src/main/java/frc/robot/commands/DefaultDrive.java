package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {

    private final DriveSubsystem chassis;
    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rotation_supplier;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);

    public DefaultDrive(DriveSubsystem chassis, ChassisSpeeds chassisSpeeds) {
        this(chassis, () -> chassisSpeeds.vxMetersPerSecond, () -> chassisSpeeds.vyMetersPerSecond, () -> chassisSpeeds.omegaRadiansPerSecond);
    }

    public DefaultDrive(DriveSubsystem chassis, CommandXboxController controller) {
        this(chassis, () -> -modifyAxis(controller.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getRightX())* DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }
  
    public DefaultDrive(DriveSubsystem driveSubsystem,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier) {
        
        this.chassis = driveSubsystem;
        this.x_supplier = translationXSupplier;
        this.y_supplier = translationYSupplier;
        this.rotation_supplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }
    
    @Override
    public void execute() {
        double xSpeed = xLimiter.calculate(x_supplier.getAsDouble());
        double ySpeed = yLimiter.calculate(y_supplier.getAsDouble());
        double rotationSpeed = rotation_supplier.getAsDouble() * 0.7;
        
        ChassisSpeeds output = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        Pose2d pose;
        double[] point;
        Translation2d tr;
        Rotation2d distAngle;
        Translation2d dist2d;
        Rotation2d adjustmentAngle;

        switch (chassis.drive_mode) {
            case 1: // Field-Oriented
            output = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, chassis.rotation());
            break;
            
            case 2: // Fixed-Point Tracking
            point = SmartDashboard.getNumberArray("TargetPose", (double[]) null);
            pose = chassis.getPose();
            dist2d = pose.getTranslation().minus(new Translation2d(point[0], point[1]));
            distAngle = new Rotation2d(Math.atan2(dist2d.getY(), dist2d.getX()));
            adjustmentAngle = pose.getRotation().plus(distAngle);
            tr = new Translation2d(xSpeed, ySpeed).rotateBy(adjustmentAngle.unaryMinus());
            output = new ChassisSpeeds(tr.getX(), tr.getY(), rotationSpeed);
            break;

            case 3: // Fixed Alignment
            point = SmartDashboard.getNumberArray("TargetPose", (double[]) null);
            pose = chassis.getPose();
            dist2d = pose.getTranslation().minus(new Translation2d(point[0], point[1]));
            distAngle = new Rotation2d(Math.atan2(dist2d.getY(), dist2d.getX()));
            adjustmentAngle = pose.getRotation().plus(distAngle);
            tr = new Translation2d(0, ySpeed).rotateBy(adjustmentAngle.unaryMinus());
            output = new ChassisSpeeds(tr.getX(), tr.getY(), 0);
            break;
        }

        chassis.drive(output);
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.05); // Deadband
        value = Math.copySign(value * value, value); // Square the axis
        return value;
    }
}