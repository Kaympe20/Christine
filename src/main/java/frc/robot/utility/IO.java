package frc.robot.utility;

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
    
    //public final IntakeNeo intake = new IntakeNeo();
    
    public final Limelight limelight = new Limelight();

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
        // intake.setDefaultCommand( new InstantCommand( () -> {
        //     intake.setVoltage(driveController.getLeftY() * 6);
        // }));
        driveController.a().onTrue(new Aimbot(this));
        driveController.b().onTrue(new InstantCommand(chassis::resetSteerPositions));
        driveController.x().onTrue(new InstantCommand(chassis::syncEncoders));
        driveController.y().toggleOnTrue(new DistanceDrive(this, 2.3));
        // //driveController.y().onTrue(autoSelector.getSelected());

        // driveController.leftBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = 0));
        // driveController.rightBumper().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Field_Oriented));
        // driveController.leftTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Point_Tracking));
        // driveController.rightTrigger().onTrue(new InstantCommand(() -> chassis.drive_mode = DriveConstants.Fixed_Alignment));

        driveController.povDownLeft().onTrue(new InstantCommand(chassis::resetAbsolute));
        driveController.povUpLeft().onTrue(new InstantCommand(chassis::disableChassis));
        driveController.povDownRight().onTrue(new InstantCommand(chassis::activeChassis));
        driveController.back().onTrue(new InstantCommand(chassis::resetOdometry));
    }
}
