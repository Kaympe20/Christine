// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Flywheel;
import frc.robot.utility.DebugTable;
import frc.robot.utility.IO;

public class AutoFire extends SequentialCommandGroup {

  public AutoFire(IO io, boolean pass_under) {
    ProfiledShooter profiledShoot = new ProfiledShooter(io, Flywheel.PASS_OFF_ANGLE);
    addRequirements(io.intake, io.shooter, io.shooter_light);
    addCommands(
        new RepeatCommand(
            new SequentialCommandGroup(
                new PassOff(io),

                new WaitUntilCommand(() -> io.chassis.distance(io.shooter_light.tagPose()) < (double) DebugTable.get("RampUp Distance", 2.5)), // REPLACE: 2.5
                new InstantCommand(() -> io.shooter.flywheelVoltage(-16)),

                new WaitUntilCommand(() -> io.chassis.distance(io.shooter_light.tagPose()) < (double) DebugTable.get("Angle Setting Distance", 2.5)), // REPLACE: 2.5
                new InstantCommand(() -> profiledShoot.setAngle(Flywheel.PASS_OFF_ANGLE)),

                new WaitUntilCommand(() -> io.chassis.distance(io.shooter_light.tagPose()) < (double) DebugTable.get("ShootingDistance", 2.5)), // REPLACE: 2.5
                
                // Shooting
                new InstantCommand(() -> io.shooter.helperVoltage(12)),
                new InstantCommand(() -> io.intake.speed(-1)),
                new WaitCommand(0.2),
                new InstantCommand(() -> {
                  if (pass_under) {
                    profiledShoot.stop();
                    io.shooter.flywheelVoltage(0.0);
                  }

                  io.shooter.helperVoltage(0.0);
                  io.intake.speed(0.0);
                }))));
  }
}
