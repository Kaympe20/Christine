// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.IO;

public class DistanceDrive extends Command {
  
  IO io;
  double distance;

  public DistanceDrive(IO io, double distance) {
    this.io = io;
    this.distance = distance;
    addRequirements(io.chassis, io.shooter_light);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    io.chassis.drive(new ChassisSpeeds(1.25, 0, 0));
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.stop();
  }

  @Override
  public boolean isFinished() {
    return io.shooter_light.distance() < distance;
  }
}
