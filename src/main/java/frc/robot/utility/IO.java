package frc.robot.utility;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.*;

public class IO extends SubsystemBase {
    final CommandXboxController driveController = new CommandXboxController(0);
    final CommandXboxController mechController = new CommandXboxController(1);

    public final Swerve chassis = new Swerve();
//     public final LEDs leds = n       ew LEDs();
    public final Intake intake = new Intake();
    public final Limelight shooter_light = new Limelight("shooter");
    public final Limelight intake_light = new Limelight("intake");
    public final Flywheel shooter = new Flywheel();
    public final Climber climber = new Climber();
    public final ProfiledShooter profiledShoot = new ProfiledShooter(this, 64);

    public CommandScheduler scheduler = CommandScheduler.getInstance();

    public IO(SendableChooser<Runnable> bindings) {
        bindings.setDefaultOption("Testing", this::configTesting);
        bindings.addOption("Manual", this::configManual);
    }

    public void configGlobal() {
        chassis.setDefaultCommand(new DefaultDrive(this, driveController));
        shooter.setDefaultCommand(profiledShoot);

        driveController.leftBumper()
                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.ROBOT_ORIENTED));
        driveController.rightBumper()
                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED));
        driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.SLOW))
                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));
        driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.TURBO))
                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));

        driveController.a().onTrue(new InstantCommand(() -> climber.setHangVolts(-6)))
                .onFalse(new InstantCommand(() -> climber.setHangVolts(0)));
        driveController.y().onTrue(new InstantCommand(() -> climber.setHangVolts(6)))
                .onFalse(new InstantCommand(() -> climber.setHangVolts(0)));
        driveController.x().onTrue(new InstantCommand(() -> climber.setElevatorVolts(10)))
                .onFalse(new InstantCommand(() -> climber.setElevatorVolts(0)));
        driveController.b().onTrue(new InstantCommand(() -> climber.setElevatorVolts(-4)))
                .onFalse(new InstantCommand(() -> climber.setElevatorVolts(0)));

        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));
        // driveController.start().onTrue(new InstantCommand(() ->
        // chassis.resetOdometry(shooter_light.poseEstimation(chassis.rotation()))));
        driveController.start().onTrue(new InstantCommand(climber::resetEncoders));
        driveController.povUpRight().onTrue(new InstantCommand(scheduler::cancelAll));
        driveController.povRight().onTrue(new Aimbot(this));
        // driveController.leftTrigger().onTrue(new
        // InstantCommand(profiledShoot::stop));

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void configManual() {
        // driveController.a().onTrue(new Endgame(this));

        mechController.leftBumper().onTrue(new InstantCommand(() -> intake.speed(-1)))
                .onFalse(new InstantCommand(() -> {
                    intake.speed(0);
                    profiledShoot.
                    stop();
                }));
        mechController.rightBumper().onTrue(new InstantCommand(() -> intake.speed(0.5)))
                .onFalse(new InstantCommand(() -> intake.speed(0)));

        mechController.rightTrigger().onTrue(new AmpShooting(this));
        mechController.leftTrigger().onTrue(new CloseUpShooting(this));

        mechController.back().onTrue(new InstantCommand(scheduler::cancelAll));
        // mechController.x().onTrue(new InstantCommand(scheduler::cancelAll));

        mechController.y().onTrue(
                new InstantCommand(() -> profiledShoot.setAngle((double) DebugTable.get("Test Angle", Flywheel.AMP)))); // 86
                                                                                                                        // for
                                                                                                                        // distance
        mechController.a().onTrue(new PassOff(this));
        mechController.b().onTrue(new InstantCommand(() -> {
            profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE);
            shooter.flywheelVoltage((double) DebugTable.get("Test Flywheel Voltage", -16.0));
            shooter.helperVoltage((double) DebugTable.get("Test Helper Voltage", 12.0));
        })).onFalse(new InstantCommand(() -> {
            intake.speed(0);
            shooter.flywheelVoltage(0);
            shooter.helperVoltage(0);
        }));

        mechController.x().onTrue(new Trap(this));

        mechController.povLeft().onTrue(new InstantCommand(() -> {
                profiledShoot.setAngle(220);
                climber.setHangPos(Climber.HANG_UP_POS);
        }));
        mechController.povUp().onTrue(new InstantCommand(() -> climber.setHangPos(Climber.HANG_DOWN_POS)));
        mechController.povRight().onTrue(new InstantCommand(() -> climber.setElevatorPos(Climber.ELEVATOR_UP_POS)));
        mechController.povDown().onTrue(new InstantCommand(() -> climber.setElevatorPos(Climber.ELEVATOR_DOWN_POS)));

        // mechController.povDown().onTrue(new ToggleIntake(this));
        // mechController.povUp().onTrue(new InstantCommand(profiledShoot::stop));
        // mechController.povLeft().onTrue(new InstantCommand(() -> {
        // intake.speed(-1);
        // profiledShoot.stop();
        // })).onFalse(new InstantCommand(() -> intake.speed(0)));
        // //mechController.povUp().onTrue(new InstantCommand(() ->
        // shooter.helperVoltage((double) DebugTable.get("Test Helper Voltage",
        // -12.0)))).onFalse(new InstantCommand(() -> shooter.helperVoltage(0)));
        // mechController.povRight().onTrue(new InstantCommand(() ->
        // profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 75.0))));
    }

    public void configTesting() {
        // driveController.y().onTrue(new InstantCommand(() ->
        // autoSelector.getSelected().schedule()));

        driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disable));
        driveController.povDownRight().onTrue(new InstantCommand(chassis::enable));

        mechController.leftBumper().onTrue(new InstantCommand(() -> shooter.pivotVoltage(1.5)))
                .onFalse(new InstantCommand(() -> shooter.pivotVoltage(0)));
        mechController.rightBumper().onTrue(new InstantCommand(() -> shooter.pivotVoltage(-1.5)))
                .onFalse(new InstantCommand(() -> shooter.pivotVoltage(0)));

        mechController.leftTrigger().onTrue(new ToggleIntake(this));
        mechController.rightTrigger().onTrue(new InstantCommand(() -> {
            shooter.flywheelVoltage(-16);
            shooter.helperVoltage(-6);
        })).onFalse(new InstantCommand(() -> {
            intake.speed(0);
            shooter.flywheelVoltage(0);
            shooter.helperVoltage(0);
        }));

        mechController.a()
                .onTrue(new InstantCommand(() -> profiledShoot.setAngle((double) DebugTable.get("Test Angle", 200.0))));
        mechController.b().onTrue(new InstantCommand(profiledShoot::stop));

        mechController.povLeft().onTrue(new InstantCommand(() -> climber.setElevatorVolts(3)));
        mechController.povRight().onTrue(new InstantCommand(() -> climber.setElevatorVolts(-3)));
        mechController.povUp().onTrue(new InstantCommand(() -> climber.setHangVolts(3)));
        mechController.povDown().onTrue(new InstantCommand(() -> climber.setElevatorVolts(-3)));
    }

    StructPublisher<Pose2d> estimated_pose = NetworkTableInstance.getDefault().getTable("Debug")
            .getStructTopic("Estimated Pose", Pose2d.struct).publish();

    @Override
    public void periodic() {
        estimated_pose.set(shooter_light.poseEstimation(chassis.rotation()));
        SmartDashboard.putNumber("Odometry Distance",
                chassis.distance(new Pose2d(shooter_light.tagPose()[0], shooter_light.tagPose()[2], new Rotation2d())));
    }
}