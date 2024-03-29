// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.utility.IO;

public class AutoFire extends SequentialCommandGroup {

  public AutoFire(IO io, boolean pre_ramped) {

    addRequirements(io.intake, io.shooter, io.shooter_light);
    addCommands(
        new RepeatCommand(
          new ConditionalCommand(
                new SequentialCommandGroup(
                  new PassOff(io),
                  new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),
                  new InstantCommand(() -> io.profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE)),
                  new WaitUntilCommand(() -> io.chassis.distance(new Pose2d(io.shooter_light.tagPose()[0], io.shooter_light.tagPose()[2], new Rotation2d())) < 1.5),
                  new Shoot(io)), 
                new SequentialCommandGroup(
                  new PassOff(io),
                  new WaitUntilCommand(() -> io.chassis.distance(new Pose2d(io.shooter_light.tagPose()[0], io.shooter_light.tagPose()[2], new Rotation2d())) < 2.0),
                  new CloseUpShooting(io)),
                ()-> pre_ramped)));
  }
}
