// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public class ExampleSubsystem extends SubsystemBase {
   //shawn auton stuff
  //Gyro sensor
    private final Gyro m_gyro = new ADXRS450_Gyro();

    //Odometry for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;
  
    //good normal stuff
  public static final double normalSpeedMult = DriveConstants.kNormSpeedMult;
  public static final double turboSpeedMult = DriveConstants.kTurboSpeedMult;
  public long start = System.currentTimeMillis();

  private WPI_TalonSRX leftFrontDrive = new WPI_TalonSRX(DriveConstants.kLeftFrontPort);
  private WPI_TalonSRX leftRearDrive = new WPI_TalonSRX(DriveConstants.kLeftRearPort);
  public WPI_TalonSRX rightFrontDrive = new WPI_TalonSRX(DriveConstants.kRightFrontPort);
  public WPI_TalonSRX rightRearDrive = new WPI_TalonSRX(DriveConstants.kRightRearPort);
  private CANSparkMax leftTank1 = new CANSparkMax(DriveConstants.kLeftSpark1Port, MotorType.kBrushless);
  private CANSparkMax leftTank2 = new CANSparkMax(DriveConstants.kLeftSpark2Port, MotorType.kBrushless);
  private CANSparkMax rightTank1 = new CANSparkMax(DriveConstants.kRightSpark1Port, MotorType.kBrushless);
  private CANSparkMax rightTank2 = new CANSparkMax(DriveConstants.kRightSpark2Port, MotorType.kBrushless);

  public MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftTank1, leftTank2);
  public MotorControllerGroup m_rightMotors = new MotorControllerGroup(rightTank1, rightTank2);
  private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
//Had to switch places of left and right motors to allow strafe
  private MecanumDrive m_xDrive = new MecanumDrive(leftFrontDrive, leftRearDrive, rightFrontDrive, rightRearDrive);
  private Encoder encoderR = new Encoder(0, 1);
  private Encoder encoderL = new Encoder(2, 3);
  
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
   
      //sets distance per pulse for encoders
      encoderL.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
      encoderR.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

      //resets encoders then sets odometry
      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), encoderL.getDistance(), encoderR.getDistance());
  }
  //resets the encoders of the robot
  public void resetEncoders()
  {
      encoderL.reset();
      encoderR.reset();
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

  @SuppressWarnings("ParameterName")
  public void xDriveCommand(double stickX, double stickY, double stickZ){
    rightFrontDrive.setInverted(stickX != 0);
    rightRearDrive.setInverted(stickX != 0);
    
    m_xDrive.driveCartesian(-stickY * DriveConstants.kNormSpeedMult, stickX * DriveConstants.kNormSpeedMult, stickZ * DriveConstants.kNormSpeedMult);
  }



  //auton shuff
  
   //returns the pose of the robot
   public Pose2d getPose()
   {
       return m_odometry.getPoseMeters();
   }

   //returns the speed of the wheels
   public DifferentialDriveWheelSpeeds getWheelSpeeds()
   {
       return new DifferentialDriveWheelSpeeds(encoderL.getRate(), encoderR.getRate());
   }

   //resets the odometrys pose
   public void resetOdometry(Pose2d pose)
   {
       resetEncoders();
       m_odometry.resetPosition(m_gyro.getRotation2d(), encoderL.getDistance(), encoderR.getDistance(), pose);
   }

     //Drives the robots left and right side VIA volts
     public void tankDriveVolts(double leftVolts, double rightVolts)
     {
        float speedEdit = 0.66f;
        float speedEditRight = 0.55f;
         m_leftMotors.setVoltage(leftVolts * speedEdit);
         m_rightMotors.setVoltage(-rightVolts * speedEditRight);
         m_drive.feed();
     }
}
