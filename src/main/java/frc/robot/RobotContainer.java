// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Constants.Paths;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos = new SendableChooser<Command>();

  public IO io = new IO(bindings, autos);

  public RobotContainer() {
    io.configGlobal();
    io.configTesting();
    addAutos();
  }

  public void addAutos(){
    NamedCommands.registerCommand("Pickup", new PrintCommand("Picking Up!!"));
    NamedCommands.registerCommand("ScoreInAmp", new PrintCommand("Scoring in amp!!"));
    NamedCommands.registerCommand( "Score", new PrintCommand("Scoring in Subwoffer!!"));
    autos.addOption("all", AutoBuilder.followPath(Paths.all));
  }

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
