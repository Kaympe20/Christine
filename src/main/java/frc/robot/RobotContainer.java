// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AmpShooting;
import frc.robot.commands.AutoFiring;
import frc.robot.commands.CloseUpShooting;
import frc.robot.commands.PassOff;
import frc.robot.commands.ProfiledShooter;
import frc.robot.subsystems.Swerve.DriveConstants;
import frc.robot.utility.IO;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos;
  HashMap<String, Command> commands = new HashMap<>();

  public IO io = new IO(bindings, autos);

  public RobotContainer() {
    addAutos();

    autos = AutoBuilder.buildAutoChooser("Speaker Rings Centre");
 
    SmartDashboard.putData("Autos",autos);
    SmartDashboard.putData("Bindings", bindings);

    io.configGlobal();
    io.configTesting();
  }

  public void addAutos(){
    commands.put("pickup", new AutoFiring(io));
    commands.put("firing", new AutoFiring(io));
    commands.put("ScoreInamp", new AmpShooting(io));
    commands.put( "score", new CloseUpShooting(io));
    commands.put( "Profiled Arm", io.profiledShoot);
    
    NamedCommands.registerCommands(commands);
  }

  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(() -> io.chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED);
    return new InstantCommand(() -> io.chassis.DRIVE_MODE = DriveConstants.FIELD_ORIENTED).andThen(autos.getSelected());
  }
}