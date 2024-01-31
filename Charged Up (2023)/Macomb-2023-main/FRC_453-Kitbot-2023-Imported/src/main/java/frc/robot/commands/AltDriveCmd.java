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
public class AltDriveCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem m_subsystem;
  private final Joystick operator = new Joystick(Constants.IOConstants.kOperatorControllerPort);
  private final Joystick driver = new Joystick(Constants.IOConstants.kDriverControllerPort);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AltDriveCmd(ExampleSubsystem subsystem) {
    m_subsystem = subsystem;
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
    
    if(Robot.m_robotContainer.curDrive == false){
      m_subsystem.tankDriveCommand(driver.getRawAxis(2), driver.getRawAxis(1));
      //Robot.m_robotContainer.m_exampleSubsystem.setDefaultCommand(Robot.m_robotContainer.m_defDrive);
      if(Robot.m_robotContainer.allDrive == true){
        m_subsystem.xDriveCommand(0, driver.getY(), driver.getZ());
      }
    }
    else{
      m_subsystem.xDriveCommand(driver.getX(), driver.getY(), driver.getZ());
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

  public double getSpeed(boolean isRight) {
    //If driver stick is pushed forward or backward, return the speed no matter what side
    if(driver.getRawAxis(1) > 0 || driver.getRawAxis(1) < 0) {
        return driver.getRawAxis(1);
    }
    //If driver rotates on z-axis to the right
    else if(driver.getRawAxis(0) > 0){
        //Check if the side asked about is the right wheels or left and return appropriate values
        if(isRight) {
            return -driver.getRawAxis(0);
        }
        else {
            return driver.getRawAxis(0);
        }
    }
    //If driver roates on z-axis to the left
    else if(driver.getRawAxis(0) < 0){
        //Check if the side asked about is the right wheels or left and return appropriate values
        if(isRight) {
            return driver.getRawAxis(0);
        }
        else {
            return -driver.getRawAxis(0);
        }
    }
    //No driver input returns no speed
    else {
        return 0;
    }

  }
}
