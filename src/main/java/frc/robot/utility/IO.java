package frc.robot.utility;


import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.*;

public class IO extends SubsystemBase{
    final CommandXboxController driveController = new CommandXboxController(0);
    final CommandXboxController mechController = new CommandXboxController(1);

    public final Swerve chassis = new Swerve();
    public Orchestra play = new Orchestra();
    public final LEDs leds = new LEDs();
    public final Intake intake = new Intake();
    public final Limelight limelight = new Limelight();
    public final Flywheel shooter = new Flywheel();
    public final ProfiledShooter profiledShoot = new ProfiledShooter(this, 64);

    SendableChooser<Command> autoSelector;

    public IO(SendableChooser<Runnable> bindings, SendableChooser<Command> selector){
        bindings.setDefaultOption("Testing", this::configTesting);
        bindings.addOption("Manual", this::configManual);
        autoSelector = selector;
    }

    public void configGlobal(){
        chassis.setDefaultCommand(new DefaultDrive(this, driveController));
        shooter.setDefaultCommand(profiledShoot);

        driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.ROBOT_ORIENTED));
        driveController.rightBumper().onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED));
        // driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIXED_POINT_TRACKING));
        // driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIXED_ALIGNMENT));


        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));
        driveController.start().onTrue(new InstantCommand(() -> chassis.resetOdometry(limelight.poseEstimation(chassis.rotation()))));

        // driveController.a().onTrue(new SequentialCommandGroup(
        //     new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED),
        //       autoSelector.getSelected() ));

        driveController.povUpRight().onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        // leds.sequenceLed();
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void configManual(){
        mechController.rightBumper().onTrue(new InstantCommand(() -> intake.speed(0.5))).onFalse(new InstantCommand(() -> intake.speed(0)));
        mechController.leftBumper().onTrue(new InstantCommand(() -> intake.speed((double) DebugTable.get("Test Intake Voltage", -12.0)))).onFalse(new InstantCommand(() -> shooter.helperVoltage(0)));
     
        mechController.rightTrigger().onTrue(new AmpShooting(this));
        mechController.leftTrigger().onTrue(new CloseUpShooting(this));
        // mechController.leftTrigger().onTrue(new InstantCommand(() -> {
        //     shooter.flywheelSpeed(1);
        //     shooter.helperVoltage((double) DebugTable.get("Test Helper Voltage", -6.0));
        // })).onFalse(new InstantCommand(() ->{
        //     intake.speed(0);
        //     shooter.flywheelSpeed(0);
        //     shooter.helperVoltage(0);
        // }));
        mechController.back().onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        mechController.x().onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll));
        mechController.y().onTrue(new InstantCommand(() -> profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 75.0))));
        mechController.a().onTrue(new PassOff(this));
        mechController.b().onTrue(new InstantCommand(() -> {
            shooter.flywheelVoltage((double) DebugTable.get("Test Flywheel Voltage", -16.0));
            shooter.helperVoltage((double) DebugTable.get("Test Helper Voltage", -6.0));
        })).onFalse(new InstantCommand(() ->{
            intake.speed(0);
            shooter.flywheelVoltage(0);
            shooter.helperVoltage(0);
        }));

        mechController.povDown().onTrue(new ToggleIntake(this));
        mechController.povUp().onTrue(new InstantCommand(profiledShoot::stop));
        mechController.povLeft().onTrue(new InstantCommand(() -> intake.speed((double) DebugTable.get("Test Intake Voltage", -1.0)))).onFalse(new InstantCommand(() -> shooter.helperVoltage(0)));
        //mechController.povUp().onTrue(new InstantCommand(() -> shooter.helperVoltage((double) DebugTable.get("Test Helper Voltage", -12.0)))).onFalse(new InstantCommand(() -> shooter.helperVoltage(0)));
        mechController.povRight().onTrue(new InstantCommand(() -> profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 75.0))));
    }

    public void configTesting(){
        // driveController.y().onTrue(new InstantComm   and(() -> autoSelector.getSelected().schedule()));
        
        driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disableChassis));
        driveController.povDownRight().onTrue(new InstantCommand(chassis::activeChassis));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disableChassis));


        mechController.leftBumper().onTrue(new InstantCommand(() -> shooter.pivotVoltage(1.5))).onFalse(new InstantCommand(() -> shooter.pivotVoltage(0)));
        mechController.rightBumper().onTrue(new InstantCommand(() -> shooter.pivotVoltage(-1.5))).onFalse(new InstantCommand(() -> shooter.pivotVoltage(0))); 

        //mechController.leftTrigger().onTrue(new ToggleIntake(this)); 
        mechController.rightTrigger().onTrue(new InstantCommand(() -> {
            shooter.flywheelVoltage(-16);
            shooter.helperVoltage(-6);
        })).onFalse(new InstantCommand(() ->{
            intake.speed(0);
            shooter.flywheelVoltage(0);
            shooter.helperVoltage(0);
        })); 

        mechController.a().onTrue(new InstantCommand(() -> profiledShoot.setAngle(150)));
        // mechController.x().onTrue(new InstantCommand(() -> profiledShoot.setAngle(200)));
        mechController.y().onTrue(new InstantCommand(() -> profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 200.0))));
        // mechController.leftBumper().onTrue(new InstantCommand(() -> intake.speed(-1))).onFalse(new InstantCommand(() -> intake.speed(0)));
        mechController.b().onTrue(new InstantCommand(profiledShoot::stop));
    }

    StructPublisher<Pose2d> estimated_pose = NetworkTableInstance.getDefault().getTable("Debug").getStructTopic("Estimated Pose", Pose2d.struct).publish();


    @Override
    public void periodic(){
        estimated_pose.set(limelight.poseEstimation(chassis.rotation()));
        SmartDashboard.putNumber("Odometry Distance", chassis.distance( new Pose2d(limelight.tagPose()[0], limelight.tagPose()[2], new Rotation2d())));
    }
}