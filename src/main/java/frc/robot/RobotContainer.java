// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.utility.IO;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos;
  HashMap<String, Command> commands = new HashMap<>();

  public IO io = new IO(bindings, autos);

  public RobotContainer() {
    addAutos();

    autos = AutoBuilder.buildAutoChooser("Bismillah");
 
    SmartDashboard.putData("Autos",autos);
    SmartDashboard.putData("Bindings", bindings);

    io.configGlobal();
    io.configTesting();
  }

  public void addAutos(){
    commands.put("Pickup", new InstantCommand(() -> SmartDashboard.putBoolean("Pickup", true)));
    commands.put("ScoreInAmp", new PrintCommand("Scoring in amp!!"));
    commands.put( "Score", new PrintCommand("Scoring in Subwoffer!!"));
    
    NamedCommands.registerCommands(commands);
    
    // autos.setDefaultOption("Bismillah", AutoBuilder.buildAuto("Bismillah"));
    // autos.addOption("Triple Subwoffer", AutoBuilder.buildAuto("Triple score in subwofer"));
    // autos.addOption("Scoring on the way out", AutoBuilder.buildAuto("Scoring on the way out"));
    // autos.addOption("Double Amp", AutoBuilder.buildAuto("Double score in amp"));
    // autos.addOption("Double subwofer", AutoBuilder.buildAuto("Double score in subwoffer"));
    // autos.addOption("Do everything", AutoBuilder.buildAuto("Do everything"));
    // autos.addOption("Do everything and otherside", AutoBuilder.buildAuto("Do everything w other side move"));
  }


  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
