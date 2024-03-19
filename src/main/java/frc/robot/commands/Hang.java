// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.utility.IO;

public class Hang extends PIDCommand {

  IO io;
  
  public static double TOP_HANG_POSITION = 50; //TODO: PLACEHOLDER
  public static double ZERO_POSITION = 0; //TODO: PLACEHOLDER
  public static double MINIMUM_ADJUSTMENT = 2;

  public Hang(IO io) { //ONLY USE IF ON BOARD PID CONTROL DOES NOT WORK AS INTEDED ON TALONFX
    super(
        new PIDController(1, 0, 0),
        () -> io.climber.getHangPos(),
        () -> (Math.abs(io.climber.getHangPos() - TOP_HANG_POSITION) < MINIMUM_ADJUSTMENT ? ZERO_POSITION : TOP_HANG_POSITION),
        output -> {
          io.climber.setHangVolts(output);
        });
        this.io = io;
        addRequirements(io.climber);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < 2;
  }

  @Override
  public void end(boolean interrupted) {
    io.climber.hangStop();
  }
}
