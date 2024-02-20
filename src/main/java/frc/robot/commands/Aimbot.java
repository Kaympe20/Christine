// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utility.Constants;
import frc.robot.utility.IO;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.wpilibj.DriverStation;

public class Aimbot extends PIDCommand {
  
  //public static double AIMBOT_OFFSET_FORWARD = 5.67; // TODO: PLACEHOLDER
  //public static double AIMBOT_OFFSET_BACKWARD = -11.13; // TODO: PLACEHOLDER
  public static double AimbotSpeed = 0.5; // TODO: PLACEHOLDER
  public static double minimumAdjustment = 2.5; // TODO: PLACEHOLDER

  IO io;

  public Aimbot(IO io) {
    super(
        new PIDController(1, 0, 0), // TODO: PLACEHOLDER
        () -> io.limelight.targetData().horizontalOffset,
        () -> 0,
        output -> {
          io.chassis.drive(new ChassisSpeeds(0, 0, output * AimbotSpeed * Constants.DRIVE_MAX_VELOCITY_METERS_PER_SECOND));
        });
        this.io = io;
        addRequirements(io.chassis, io.limelight);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < minimumAdjustment || io.limelight.targetData().hasTargets;
  }

  @Override
  public void end(boolean interrupted) {
    io.chassis.stop();
  }
}
