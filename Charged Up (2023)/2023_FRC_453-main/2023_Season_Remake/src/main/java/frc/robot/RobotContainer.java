// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.security.KeyStore.TrustedCertificateEntry;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

// The robot's subsystems
    public final Climber m_climber = new Climber();
    public final Shooter m_shooter = new Shooter();
    public final Lifter m_lifter = new Lifter();
    public final Intake m_intake = new Intake();
    public final Drivetrain m_drivetrain = new Drivetrain();
    public final LimeLight m_limelight = new LimeLight();

// Joysticks
private final Joystick operator = new Joystick(OIConstants.kOperatorControllerPort);
private final Joystick driver = new Joystick(OIConstants.kDriverControllerPort);

private final AutoDrive m_commandAutoDrive = new AutoDrive(m_limelight, m_drivetrain);

  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  public long startTime = 0;

  /**
  * The container for the robot.  Contains subsystems, OI devices, and commands.
  */
  private RobotContainer() {

    configureButtonBindings();

     // Configure default commands
        m_drivetrain.setDefaultCommand(
            new RunCommand(
                () ->
                    m_drivetrain.drive(
                        driver.getX(),
                        driver.getY(),
                        -driver.getZ(),
                        false, false), 
                        m_drivetrain)
        );

    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Auton 1", new RunCommand(
        () ->
            m_drivetrain.tarmacDrive(System.currentTimeMillis()), 
                m_drivetrain));

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
// Create some buttons

  }

public Joystick getDriver() {
        return driver;
    }

public Joystick getOperator() {
        return operator;
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public Command getAutonomousCommand() {
    /* // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            exampleTrajectory,
            m_drivetrain::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kDriveKinematics,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            m_drivetrain::getCurrentWheelSpeeds,
            m_drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()); */
        
    // Run path following command, then stop at the end.
    return new RunCommand(
        () ->
            m_drivetrain.tarmacDrive(startTime), 
                m_drivetrain);
  }
  

}
