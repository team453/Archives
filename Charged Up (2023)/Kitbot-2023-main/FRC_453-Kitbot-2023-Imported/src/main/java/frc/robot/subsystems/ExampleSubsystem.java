// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class ExampleSubsystem extends SubsystemBase {
  public static final double normalSpeedMult = DriveConstants.kNormSpeedMult;
  public static final double turboSpeedMult = DriveConstants.kTurboSpeedMult;
  public long start = System.currentTimeMillis();

  private WPI_TalonSRX leftFrontDrive = new WPI_TalonSRX(DriveConstants.kLeftFrontPort);
  private WPI_TalonSRX leftRearDrive = new WPI_TalonSRX(DriveConstants.kLeftRearPort);
  private WPI_TalonSRX rightFrontDrive = new WPI_TalonSRX(DriveConstants.kRightFrontPort);
  private WPI_TalonSRX rightRearDrive = new WPI_TalonSRX(DriveConstants.kRightRearPort);

  public MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftFrontDrive, leftRearDrive);
  public MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightFrontDrive, rightRearDrive);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @SuppressWarnings("ParameterName")
  public void tankDriveCommand(double speedRight, double speedLeft){
    if(Robot.m_robotContainer.curDrive == false){
        m_drive.arcadeDrive(speedRight * DriveConstants.kNormSpeedMult, +speedLeft * DriveConstants.kNormSpeedMult);
    }
    else{
      m_drive.tankDrive(-speedRight * DriveConstants.kNormSpeedMult, speedLeft * DriveConstants.kNormSpeedMult);
    }
  }

}
