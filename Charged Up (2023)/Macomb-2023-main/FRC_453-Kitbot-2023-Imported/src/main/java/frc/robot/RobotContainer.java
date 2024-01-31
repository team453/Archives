// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.commands.SwapDriveCmd;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.commands.AllDriveCmd;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;


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

//Adding imports to reduce clutter
import frc.robot.Constants.*;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
 
  //public final Arm m_arm = new Arm();
  public static final PneumaticsSubsystem m_pneumatics = new PneumaticsSubsystem();
  public final ExampleSubsystem m_drivetrain = new ExampleSubsystem();
  public boolean curDrive = true; //True = X Drive, False = Tank Drive
  public boolean allDrive = false;
  private final AltDriveCmd m_driveCmd = new AltDriveCmd(m_drivetrain);
  private final SwapDriveCmd m_swapDriveCmd = new SwapDriveCmd(m_drivetrain);
  //public final DriveForward c_autonCmd = new DriveForward(m_drivetrain, "forward");

  //private final OpenClawCmd openClaw = new OpenClawCmd();
  //private final CloseClawCmd closeClaw = new CloseClawCmd();
  //private final resetArmCmd resetArmPosition = new resetArmCmd();
  private final AllDriveCmd c_allDriveSwap = new AllDriveCmd(m_drivetrain);

  // The operator's joystick as a CommandJoystick. This is used to create commands that use the joystick
  Joystick operator = new Joystick(IOConstants.kOperatorControllerPort);
  Joystick driver = new Joystick(IOConstants.kDriverControllerPort);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  
    new JoystickButton(driver, DriveConstants.kSwapDriveMap).onTrue(m_swapDriveCmd);
    new JoystickButton(driver, 9).onTrue(c_allDriveSwap);
   //new JoystickButton(operator, 9).onTrue(m_arm.moveToBottomCommand());
   // new JoystickButton(operator, 9).onTrue(m_arm.moveToMiddleCommand());
    // new JoystickButton(operator, 10).onTrue(m_arm.moveToTopCommand());
    //new JoystickButton(operator, 7).onTrue(openClaw);
    //new JoystickButton(operator, 8).onTrue(closeClaw);

    m_drivetrain.setDefaultCommand(m_driveCmd);
  }
/* public Command getAutonomousCommand() {
  return null;
  //no please stop
} */

//start of auton
  public Command getAutonomousCommand() {
    //swap drive so tank wheels are down
    m_swapDriveCmd.execute();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kTankDriveKinematics, 7
            
            );

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory backTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            
            //move forward
            List.of(new Translation2d((0 / DriveConstants.robotConversion), (0 / DriveConstants.robotConversion)),
            new Translation2d((-0.5 / DriveConstants.robotConversion), (0 / DriveConstants.robotConversion))),
            // End meter straight ahead of where we started, facing forward
            new Pose2d((-0.5 / DriveConstants.robotConversion), 0, new Rotation2d(0)),
            // Pass config
            config);

            Trajectory forwardTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(-0.5, 0, new Rotation2d(0)),
                
                //move forward
                List.of(new Translation2d((0 / DriveConstants.robotConversion), (0 / DriveConstants.robotConversion)),
                new Translation2d((0.5 / DriveConstants.robotConversion), (0 / DriveConstants.robotConversion))),
                // End meter straight ahead of where we started, facing forward
                new Pose2d((1 / DriveConstants.robotConversion), 0, new Rotation2d(0)),
                // Pass config
                config);

    RamseteCommand ramseteCommand1 =
        new RamseteCommand(
            backTrajectory,
            m_drivetrain::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kTankDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

            RamseteCommand ramseteCommand2 =
        new RamseteCommand(
            backTrajectory,
            m_drivetrain::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kTankDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(backTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand1;
  }
//end of  auton
 
}

