package frc.robot.utility;

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
        // public final LEDs leds = new LEDs();
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
                profiledShoot.addRequirements(shooter);
                chassis.setDefaultCommand(new DefaultDrive(this, driveController));
                shooter.setDefaultCommand(profiledShoot);

                driveController.leftBumper()
                                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.ROBOT_ORIENTED));
                driveController.rightBumper()
                                .onTrue(new InstantCommand(() -> chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED));
                driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.SLOW))
                                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));
                driveController.rightTrigger()
                                .onTrue(new InstantCommand(() -> chassis.SPEED_TYPE = DriveConstants.TURBO))
                                .onFalse(new InstantCommand(() -> chassis.SPEED_TYPE = 0));

                // driveController.a().onTrue(new Aimbot(this, false));
                // driveController.b().onTrue(new Aimbot(this, true));
                driveController.x().onTrue(new InstantCommand(profiledShoot::stop));
                
                driveController.y().onTrue(new InstantCommand(() -> {
                        profiledShoot.setAngle(115.0);
                        climber.setHangPos(Climber.HANG_UP_POS);
                }));;
                driveController.a().onTrue(new InstantCommand(() -> {
                        // profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE);
                        climber.setHangPos(Climber.HANG_DOWN_POS);
                }));
                driveController.b().onTrue(new Trap(this));


                driveController.start().onTrue(new InstantCommand(climber::resetEncoders));
                driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));

                driveController.povRight().onTrue(new InstantCommand(() -> {
                        if (climber.elevatorPos() > 1)
                                climber.setElevatorVolts(-4);
                })).onFalse(new InstantCommand(() -> climber.setElevatorVolts(0)));
                
                driveController.povLeft().onTrue(new InstantCommand(() -> climber.setElevatorPos(25)));

                driveController.povUp().onTrue(new InstantCommand(() -> climber.setHangVolts(6)))
                                .onFalse(new InstantCommand(() -> climber.setHangVolts(0)));
                
                driveController.povDown().onTrue(new InstantCommand(() -> climber.setHangVolts(-6)))
                                .onFalse(new InstantCommand(() -> climber.setHangVolts(0)));

                // driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
                // driveController.povUpLeft().onTrue(new InstantCommand(chassis::disable));
                // driveController.povDownRight().onTrue(new InstantCommand(chassis::enable));

                DriverStation.silenceJoystickConnectionWarning(true);
        }

        public void configManual() {
                mechController.leftBumper().onTrue(new InstantCommand(() -> intake.speed(-.5)))
                                .onFalse(new InstantCommand(() -> {
                                        intake.speed(0);
                                        profiledShoot.stop();
                                }));
                mechController.rightBumper().onTrue(new InstantCommand(() -> intake.speed(0.3)))
                                .onFalse(new InstantCommand(() -> intake.speed(0)));

                mechController.rightTrigger().onTrue(new AmpShooting(this));
                mechController.leftTrigger().onTrue(new ToggleIntake(this));

                mechController.back().onTrue(new InstantCommand(scheduler::cancelAll));
                // mechController.start().onTrue(new InstantCommand(climber::resetEncoders));

                mechController.y().onTrue(new InstantCommand(() -> profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE)));
                //mechController.x().onTrue(new InstantCommand(() -> profiledShoot.setAngle(115.0))); // distance
                mechController.x().onTrue(new CloseUpShooting(this, profiledShoot, Flywheel.PASSING)); //passing across field
                mechController.a().onTrue(new PassOff(this, false));
                mechController.b().onTrue(new InstantCommand(() -> {
                        profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE);
                        shooter.flywheelVoltage(-14.0);
                        shooter.helperVoltage(4.0); // TESTING using smaller voltages for the helper
                })).onFalse(new InstantCommand(() -> {
                        intake.speed(0);
                        shooter.flywheelVoltage(0);
                        shooter.helperVoltage(0);
                }));

                mechController.povDown().onTrue(new ToggleIntake(this));
                mechController.povUp().onTrue(new InstantCommand(profiledShoot::stop));
                mechController.povLeft().onTrue(new InstantCommand(() -> intake.speed(-0.5))).onFalse(new InstantCommand(() -> {
                        intake.speed(0);
                        profiledShoot.stop();
                }));
                mechController.povRight().onTrue(new InstantCommand(() -> profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 75.0))));      
          }

        public void configTesting() {
                mechController.leftTrigger().onTrue(new InstantCommand(() -> chassis.setOdometry(new Pose2d(1.2, 5.53, new Rotation2d()))));
                mechController.rightTrigger().onTrue(new InstantCommand(scheduler::cancelAll));

                mechController.leftBumper().onTrue(new InstantCommand(() -> intake.pivotVoltage(2)))
                                .onFalse(new InstantCommand(() -> intake.pivotVoltage(0)));
                mechController.rightBumper().onTrue(new InstantCommand(() -> intake.pivotVoltage(-2)))
                                .onFalse(new InstantCommand(() -> intake.pivotVoltage(0)));
                                
                mechController.b().onTrue(new InstantCommand(() -> shooter.helperVoltage(4)))
                                .onFalse(new InstantCommand(() -> shooter.helperVoltage(0)));
                mechController.a().onTrue(new InstantCommand(() -> profiledShoot
                                .setAngle((double) DebugTable.get("Test Angle", Flywheel.PASS_OFF_ANGLE))));
                mechController.x().onTrue(new AutoFire(this, false));
                mechController.y().onTrue( new AutoFire(this, true));

        }

        StructPublisher<Pose2d> estimated_pose = NetworkTableInstance.getDefault().getTable("Debug")
                        .getStructTopic("Estimated Pose", Pose2d.struct).publish();

        @Override
        public void periodic() {
                estimated_pose.set(shooter_light.poseEstimation(chassis.rotation()));
                SmartDashboard.putNumber("Odometry Distance",
                                chassis.distance(new Pose2d(shooter_light.tagPose()[0], shooter_light.tagPose()[2],
                                                new Rotation2d())));
        }
}