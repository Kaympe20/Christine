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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.utility.IO;
import frc.robot.utility.Constants.Paths;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos = new SendableChooser<Command>();

  public IO io = new IO(bindings, autos);

  public RobotContainer() {
    addAutos();
    io.configGlobal();
    io.configTesting();
  }

  public void addAutos(){
    commands.put("Pickup", new PrintCommand("Picking Up!!"));
    commands.put("ScoreInAmp", new PrintCommand("Scoring in amp!!"));
    commands.put( "Score", new PrintCommand("Scoring in Subwoffer!!"));
    
    NamedCommands.registerCommands(commands);
      
    autos.setDefaultOption("all", AutoBuilder.followPath(Paths.all));
    // autos.addOption("Forward", AutoBuilder.followPath(Paths.doubleAmp));

  }

  HashMap<String, Command> commands = new HashMap<>();

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
