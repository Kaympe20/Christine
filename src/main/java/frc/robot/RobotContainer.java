// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class RobotContainer {
  SendableChooser<Runnable> bindings = new SendableChooser<Runnable>();
  SendableChooser<Command> autos = new SendableChooser<Command>();

  public IO io = new IO(bindings, autos);

  public RobotContainer() {
    io.configGlobal();
    io.configTesting();
  }

  public void addAutos(){}

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
