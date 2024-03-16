// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.utility.IO;

public class AutoFire extends SequentialCommandGroup {

  public AutoFire(IO io) {
    addRequirements(io.intake, io.shooter, io.limelight);
    addCommands(
        new RepeatCommand(
            new SequentialCommandGroup(
                new PassOff(io),
                new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
                new WaitUntilCommand(() -> io.chassis.distance(new Pose2d(io.limelight.tagPose()[0], io.limelight.tagPose()[2], new Rotation2d())) < 1.5),
                new Shoot(io))
              ));
  }
}
