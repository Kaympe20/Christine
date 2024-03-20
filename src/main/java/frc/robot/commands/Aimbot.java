// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utility.IO;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Aimbot extends PIDCommand {
  
  public static double AimbotSpeed = 0.5; // TODO: PLACEHOLDER
  public static double minimumAdjustment = 2.5; // TODO: PLACEHOLDER

  public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = 0.5;
  public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.2;

  IO io;

  public Aimbot(IO io) {
    super(
        new PIDController(1, 0, 0), // TODO: PLACEHOLDER
        () -> io.shooter_light.targetData().horizontalOffset,
        () -> 0,
        output -> {
          io.chassis.drive(new ChassisSpeeds(0, 0, output * AimbotSpeed * DRIVE_MAX_VELOCITY_METERS_PER_SECOND));
        });
        this.io = io;
        addRequirements(io.chassis, io.shooter_light);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < minimumAdjustment || io.shooter_light.targetData().hasTargets;
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.stop();
  }
}
