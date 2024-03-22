// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.utility.IO;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;


public class AdaptivePath extends Command {
  /** Creates a new AdaptivePath. */
  Command[] paths = {AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle left")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle midleft")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle mid")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle midright")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Middle right"))};
  Command[] skipPaths = {AutoBuilder.followPath(PathPlannerPath.fromPathFile("Midleft to mid")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("Mid to midright")), AutoBuilder.followPath(PathPlannerPath.fromPathFile("midright to right"))};
  IO io;

  public AdaptivePath() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    paths[0].schedule();

    for (int i = 1; i <= paths.length; i = i + 1) {
      if (io.intake_light.targetData().hasTargets) {
        paths[i].schedule();
      }
      else {
        skipPaths[i-1].schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
