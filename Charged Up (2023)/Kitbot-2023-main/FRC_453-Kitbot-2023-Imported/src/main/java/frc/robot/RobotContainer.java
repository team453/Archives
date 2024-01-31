// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ShooterConstants.OIConstants;
import frc.robot.commands.AltDriveCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.DefArmCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.RetractArmCmd;
import frc.robot.commands.SwapDriveCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
  public static ExampleCommand m_defDrive = new ExampleCommand(m_exampleSubsystem);
  public static ExtendArmCmd m_ExtendArmCmd = new ExtendArmCmd(m_pneumaticsSubsystem);
  public static RetractArmCmd m_RetractArmCmd = new RetractArmCmd(m_pneumaticsSubsystem);
  public static DefArmCmd m_defPneumatics = new DefArmCmd(m_pneumaticsSubsystem);
  public static AltDriveCmd m_altDrive = new AltDriveCmd(m_exampleSubsystem);
  public static boolean curDrive = true; //True = Default Drive, False = Alt Drive
  public static SwapDriveCmd m_swapDrive = new SwapDriveCmd(m_exampleSubsystem);

  Joystick operator = new Joystick(OIConstants.kOperatorControllerPort);
  Joystick driver = new Joystick(OIConstants.kDriverControllerPort);
  

  /* Button mapping
  * 1 - Trigger
  * 2 - Thumb
  * 3 - Bottom Left on the TOP
  * 4 - Bottom Right on the TOP
  * 5 - Top Left on the TOP
  * 6 - Top Right on the TOP
  * 7 - Top Left on the BOTTOM
  * 8 - Top Right on the BOTTOM
  * 9 - Middle Left on the BOTTOM
  * 10 - Middle Right on the BOTTOM
  * 11 - Bottom Left on the BOTTOM
  * 12 - Bottom Right on the BOTTOM
  */


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Write a default command for the subsystem
    if(curDrive){
      m_exampleSubsystem.setDefaultCommand(m_altDrive);
    }
    //m_pneumaticsSubsystem.setDefaultCommand(m_defPneumatics);
  }

  private void configureBindings() {
    // Configure button mapping here

    new JoystickButton(driver, 5).onTrue(m_ExtendArmCmd);
    new JoystickButton(driver, 3).onTrue(m_RetractArmCmd);
    //Changes current driving mode
    new JoystickButton(driver, 12).onTrue(m_swapDrive);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
