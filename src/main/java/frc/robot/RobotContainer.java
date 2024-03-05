// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AmpShooting;
import frc.robot.commands.AutoFiring;
import frc.robot.commands.CloseUpShooting;
import frc.robot.commands.PassOff;
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
    io.configManual();
  }

  public void addAutos(){
    commands.put("Pickup", new PassOff(io));
    commands.put("Firing", new AutoFiring(io));
    commands.put("ScoreInAmp", new AmpShooting(io));
    commands.put( "Score", new CloseUpShooting(io));
    
    NamedCommands.registerCommands(commands);
  }

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
