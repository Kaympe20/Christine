package frc.robot.utility;


import com.ctre.phoenix.music.Orchestra;
//import frc.robot.commands.music;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utility.Constants.DriveConstants;

public class IO {
    final CommandXboxController driveController = new CommandXboxController(0);
    final CommandXboxController mechController = new CommandXboxController(1);

    public final DriveSubsystem chassis = new DriveSubsystem();
    public Orchestra play = new Orchestra();
    public final LEDs leds = new LEDs();
    public final Intake intake = new Intake();
    public final Limelight limelight = new Limelight();
    public final Flywheel shooter = new Flywheel();
    public final ProfiledShooter profiledShoot = new ProfiledShooter(this, 300); //TODO: Change init angle

    SendableChooser<Command> autoSelector;

    public IO(SendableChooser<Runnable> bindings, SendableChooser<Command> selector){
        bindings.setDefaultOption("Testing", this::configTesting);
        bindings.addOption("Manual", this::configManual);
        autoSelector = selector;
    }

    public void configGlobal(){
        chassis.setDefaultCommand(new DefaultDrive(this, driveController));
        shooter.setDefaultCommand(profiledShoot);
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void configManual(){
        driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = 0));
        driveController.rightBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Field_Oriented));
        // driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Point_Tracking));
        // driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Alignment));

        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));

        mechController.a().onTrue(new PassOff(this, () -> profiledShoot.setAngle(shooter.PASS_OFF_ANGLE), profiledShoot::stop));
    }

    public void configTesting(){

        // driveController.a().onTrue(new InstantCommand(() -> ));
        driveController.y().onTrue(new InstantCommand(() -> autoSelector.getSelected().schedule()));
        driveController.x().onTrue(new InstantCommand(CommandScheduler.getInstance()::cancelAll));   
        
        //driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = 0));
        driveController.rightBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Field_Oriented));
        driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = 0));
        // driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Point_Tracking));
        // driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Alignment));

        driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disableChassis));
        driveController.povDownRight().onTrue(new InstantCommand(chassis::activeChassis));
        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));

        // mechController.a().onTrue(new InstantCommand(() -> shooter.setFlywheelVoltage(12))).onFalse(new InstantCommand(() -> shooter.setFlywheelVoltage(0)));
        // mechController.b().onTrue(new InstantCommand(() -> shooter.setFlywheelVoltage(-12))).onFalse(new InstantCommand(() -> shooter.setFlywheelVoltage(0)));
        // mechController.y().onTrue(new InstantCommand(() -> shooter.setHelperVoltage(8))).onFalse(new InstantCommand(() -> shooter.setHelperVoltage(0)));
        // mechController.x().onTrue(new InstantCommand(() -> shooter.setHelperVoltage(-8))).onFalse(new InstantCommand(() -> shooter.setHelperVoltage(0)));
        //mechController.leftBumper().onTrue(new InstantCommand(() -> shooter.setPivotVoltage(1.5))).onFalse(new InstantCommand(() -> shooter.setPivotVoltage(0)));
        //mechController.rightBumper().onTrue(new InstantCommand(() -> shooter.setPivotVoltage(-1.5))).onFalse(new InstantCommand(() -> shooter.setPivotVoltage(0)));
        // mechController.rightTrigger().onTrue(new InstantCommand(() -> intake.intakeVolts(1.5))).onFalse(new InstantCommand(() -> intake.intakeVolts(0)));
        // mechController.leftTrigger().onTrue(new InstantCommand(() -> intake.intakeVolts(-1.5))).onFalse(new InstantCommand(() -> intake.intakeVolts(0)));
        // mechController.povDown().onTrue(new InstantCommand(() -> intake.setVoltage(4.5))).onFalse(new InstantCommand(() -> intake.setVoltage(0)));
        // mechController.povUp().onTrue(new InstantCommand(() -> intake.setVoltage(-4.5))).onFalse(new InstantCommand(() -> intake.setVoltage(0)));

        // mechController.leftBumper().onTrue(new InstantCommand(() -> shooter.setPivotVoltage(1.5))).onFalse(new InstantCommand(() -> shooter.setPivotVoltage(0)));
        mechController.rightBumper().onTrue(new InstantCommand(() -> shooter.setPivotVoltage(-1.5))).onFalse(new InstantCommand(() -> shooter.setPivotVoltage(0))); 
       // mechController.rightBumper().onTrue(new InstantCommand(() -> profiledIntake.setAngle(24))); //jacky was here
        mechController.leftTrigger().onTrue(new ToggleIntake(this)); 
        mechController.rightTrigger().onTrue(new InstantCommand(() -> {
            shooter.setFlywheelVoltage(-16);
            shooter.setHelperVoltage(-6);
        })).onFalse(new InstantCommand(() ->{
            intake.setSpeed(0);
            shooter.setFlywheelVoltage(0);
            shooter.setHelperVoltage(0);
        })); 

        mechController.a().onTrue(new InstantCommand(() -> profiledShoot.setAngle(150)));
        mechController.x().onTrue(new InstantCommand(() -> profiledShoot.setAngle(340))); //passoff
        mechController.y().onTrue(new InstantCommand(() -> profiledShoot.setAngle( (double) DebugTable.get("Test Angle", 200.0))));
        mechController.leftBumper().onTrue(new InstantCommand(() -> intake.setSpeed(-1))).onFalse(new InstantCommand(() -> intake.setSpeed(0)));
        mechController.b().onTrue(new InstantCommand(profiledShoot::stop));
    }
}

