// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

/** An example command that uses an example subsystem. */
public class DriveForward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  private String dir = "";
  private float time;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForward(ExampleSubsystem subsystem, String direction) {
    this.m_subsystem = subsystem;
    this.dir = dir;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = System.currentTimeMillis();
    while((System.currentTimeMillis() - time) < 1000){
    if (dir.equalsIgnoreCase("forward")) {
      m_subsystem.tankDriveCommand(Constants.DriveConstants.kNormSpeedMult, Constants.DriveConstants.kNormSpeedMult);
    } else if (dir.equalsIgnoreCase("backward")) {
      m_subsystem.tankDriveCommand(-Constants.DriveConstants.kNormSpeedMult, -Constants.DriveConstants.kNormSpeedMult);
    } else if (dir.equalsIgnoreCase("left")) {
      m_subsystem.tankDriveCommand(-Constants.DriveConstants.kNormSpeedMult, Constants.DriveConstants.kNormSpeedMult);
    } else if (dir.equalsIgnoreCase("right")) {
      m_subsystem.tankDriveCommand(Constants.DriveConstants.kNormSpeedMult, -Constants.DriveConstants.kNormSpeedMult);
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

/*
 * Axis 1 = forward/backward
 */

}