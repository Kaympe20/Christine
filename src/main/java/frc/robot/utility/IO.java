package frc.robot.utility;

import javax.management.InstanceNotFoundException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utility.Constants.DriveConstants;

public class IO {
    final CommandXboxController driveController = new CommandXboxController(0);
    final CommandXboxController mechController = new CommandXboxController(1);

    public final DriveSubsystem chassis = new DriveSubsystem();
    
    public final IntakeNeo intake = new IntakeNeo();

    public final Limelight limelight = new Limelight();

    //CANSparkMax shoot = new CANSparkMax(15, MotorType.kBrushless);

    //ProfiledIntake profiled_intake = new ProfiledIntake(this, 91); // TODO: 

    SendableChooser<Command> autoSelector;

    public IO(SendableChooser<Runnable> bindings, SendableChooser<Command> selector){
        bindings.setDefaultOption("Testing", this::configTesting);
        autoSelector = selector;
    }

    public void configGlobal(){
        chassis.setDefaultCommand(new DefaultDrive(this, driveController));
        
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    public void configTesting(){
        // intake.setDefaultCommand(profiled_intake);
        // driveController.a().onTrue(new InstantCommand(profiled_intake::stop)); 
        // driveController.b().onTrue(new InstantCommand(() -> profiled_intake.setAngle(24))); //jacky was here
        // driveController.x().onTrue(new InstantCommand(() -> profiled_intake.setAngle(269)));
        // driveController.y().onTrue(new InstantCommand(() -> profiled_intake.setAngle(0))); 
        driveController.a().onTrue(new InstantCommand(() -> intake.setVoltage(12))).onFalse(new InstantCommand(()-> intake.setVoltage(0)));
        driveController.b().onTrue(new InstantCommand(() -> intake.setVoltage(-12))).onFalse(new InstantCommand(()-> intake.setVoltage(0)));

        //driveController.b().onTrue(new InstantCommand(() -> shoot.setVoltage(12))).onFalse(new InstantCommand(()-> shoot.setVoltage(0)));

        driveController.rightTrigger().onTrue(new InstantCommand(() -> intake.intakeVolts(1.5))).onFalse(new InstantCommand(() -> intake.intakeVolts(0)));
        driveController.leftTrigger().onTrue(new InstantCommand(() -> intake.intakeVolts(-1.5))).onFalse(new InstantCommand(() -> intake.intakeVolts(0)));

        driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = 0));
        driveController.rightBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Field_Oriented));
        // driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Point_Tracking));
        // driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Alignment));

        driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disableChassis));
        driveController.povDownRight().onTrue(new InstantCommand(chassis::activeChassis));
        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));
    }
}

