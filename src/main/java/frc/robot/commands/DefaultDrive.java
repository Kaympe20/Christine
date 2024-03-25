package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.utility.DebugTable;
import frc.robot.utility.IO;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends Command {

    private final IO io;
    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rotation_supplier;

    public DefaultDrive(IO io, ChassisSpeeds chassisSpeeds) {
        this(io, () -> chassisSpeeds.vxMetersPerSecond, () -> chassisSpeeds.vyMetersPerSecond, () -> chassisSpeeds.omegaRadiansPerSecond);
    }

    public DefaultDrive(IO io, CommandXboxController controller) {
        this(io, () -> -modifyAxis(controller.getLeftY()) * io.chassis.MAX_VELOCITY,
        () -> -modifyAxis(controller.getLeftX()) * io.chassis.MAX_VELOCITY,
        () -> -modifyAxis(controller.getRightX()) * io.chassis.MAX_VELOCITY);
    }
  
    public DefaultDrive(IO io,
        DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier) {
        
        this.io = io;
        this.x_supplier = translationXSupplier;
        this.y_supplier = translationYSupplier;
        this.rotation_supplier = rotationSupplier;

        addRequirements(io.chassis);
    }
    
    @Override
    public void execute() {
        double scale = (double) DebugTable.get("Translation Scale", 1.0);
        double rot_scale = (double) DebugTable.get("Rotation Scale", 0.6);
        DebugTable.set("Max Voltage", 12);

        switch(io.chassis.SPEED_TYPE){
            case DriveConstants.TURBO:
            DebugTable.set("Max Voltage", 16); // TODO: check if we should turn it down a little
            scale = 1.0;
            break;
            
            case DriveConstants.SLOW:
            DebugTable.set("Max Voltage", 12);
            scale = .25;
            rot_scale = .25;
            break;
        }

        // double xSpeed = x_supplier.getAsDouble() * scale;
        // double ySpeed = y_supplier.getAsDouble() * scale;
        // double rotationSpeed = rotation_supplier.getAsDouble() * rot_scale;

        double xSpeed = Math.pow(x_supplier.getAsDouble() * scale, 3.0);
        double ySpeed = Math.pow(y_supplier.getAsDouble() * scale, 3.0);
        double rotationSpeed = Math.pow(rotation_supplier.getAsDouble() * rot_scale, 3.0);

        
        ChassisSpeeds output = new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        Translation2d tr;
        Rotation2d adjustmentAngle;

        switch (io.chassis.DRIVE_MODE) {
            case 1: // Field-Oriented
            output = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, io.chassis.rotation());
            break;
            
            case 2: // Fixed-Point Tracking
            adjustmentAngle = io.chassis.pose().getRotation().plus(new Rotation2d(io.shooter_light.targetData().horizontalOffset));
            tr = new Translation2d(xSpeed, ySpeed).rotateBy(adjustmentAngle.unaryMinus());
            output = new ChassisSpeeds(tr.getX(), tr.getY(), rotationSpeed);
            break;

            case 3: // Fixed Alignment
            Pose2d pose = io.chassis.pose();
            adjustmentAngle = pose.getRotation().plus(new Rotation2d(io.shooter_light.targetData().horizontalOffset));
            tr = new Translation2d(0, xSpeed).rotateBy(adjustmentAngle.unaryMinus());
            output = new ChassisSpeeds(tr.getX(), tr.getY(), 0);
            break;
        }

        io.chassis.drive(output);
    }

    @Override
    public void end(boolean interrupted) {
        io.chassis.stop();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband) return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.1); // Deadband
        value = Math.copySign(value * value, value); // Square the axis
        return value;
    }
}