/* 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// This is a special import that allows us to use the commands library
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;
import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm extends SubsystemBase{
    private final WPI_TalonSRX rotationMotor = new WPI_TalonSRX(Constants.IOConstants.rotationSRXPort);
    public double targetArmPosition = 0;
    private Joystick operJoystick = new Joystick(Constants.IOConstants.kOperatorControllerPort);
   private String status = "DISABLED";
    public Arm(){
        // Set the arm motor to factory default settings
        rotationMotor.configFactoryDefault();
        
        // Set the brake mode
        rotationMotor.setNeutralMode(Constants.ArmConstants.kNeutralMode);
        
        // Set up the encoder
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1);
        rotationMotor.setSelectedSensorPosition(0);
        
        // Set the encoder phase
        rotationMotor.setSensorPhase(Constants.ArmConstants.kSensorPhase);
        
        // Set the direction of the motor
        rotationMotor.setInverted(Constants.ArmConstants.kMotorInvert);
        
        // Set the peak and nominal outputs. These are the values that the motor will be set to when the joystick is at its maximum value or normal value (Stopped)
        rotationMotor.configNominalOutputForward(0, Constants.ArmConstants.kTimeoutMs);
        rotationMotor.configNominalOutputReverse(0, Constants.ArmConstants.kTimeoutMs);
       rotationMotor.configPeakOutputForward(Constants.ArmConstants.kPeakOutput, Constants.ArmConstants.kTimeoutMs);
        rotationMotor.configPeakOutputReverse(-Constants.ArmConstants.kPeakOutput, Constants.ArmConstants.kTimeoutMs);
        // Configure the closed loop error, which is the range of error where the motor output is held constant
        rotationMotor.configAllowableClosedloopError(Constants.ArmConstants.kPIDLoopIdx, 0, Constants.ArmConstants.kTimeoutMs);
        rotationMotor.config_kF(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kF, Constants.ArmConstants.kTimeoutMs);
        rotationMotor.config_kP(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kP, Constants.ArmConstants.kTimeoutMs);
        rotationMotor.config_kI(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kI, Constants.ArmConstants.kTimeoutMs);
      rotationMotor.config_kD(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kD, Constants.ArmConstants.kTimeoutMs);

        // Setup double suppliers for pos and vel
        DoubleSupplier curArmVel = this::getArmVelocity;
        DoubleSupplier curArmPos = this::getArmPosition;

        // Set up the shuffleboard
     Shuffleboard.getTab("Arm").add("Status", status);
        Shuffleboard.getTab("Arm").addNumber("Arm Position", curArmPos);
        Shuffleboard.getTab("Arm").addNumber("Arm Velocity", curArmVel);
        Shuffleboard.getTab("Arm").addNumber("Arm Target Position", () -> targetArmPosition);
        Shuffleboard.getTab("Arm").addNumber("Arm Error", () -> targetArmPosition - curArmPos.getAsDouble());
        Shuffleboard.getTab("Arm").add(this);

        // Set the default command for the arm. This should be the command that is run when no other commands are running
        setDefaultCommand(
         // Create a new command that will use the runOnce command constructor.
             run(
            () -> {
           status = "running";
            //Check for manual control
            if (operJoystick.getRawButton(Constants.IOConstants.kArmManualControlButton)){
                // check if the arm is at the top or bottom
                if (-operJoystick.getRawAxis(Constants.IOConstants.armAxisNum) > 0)
                    //if not passed the max point
                    if(!(getArmPosition() >= Constants.ArmConstants.kMaxRotationUp))
                    {
                      // Set the arm motor to the new position based on the joystick
                         targetArmPosition = targetArmPosition + (-operJoystick.getRawAxis(Constants.IOConstants.armAxisNum) * Constants.ArmConstants.kArmSpeedMultiplier);
                         status = "moving up";
                        }
                    else
                    {
                        //STOP THAT MOTOR AAAA
                         //stop motor 
                           rotationMotor.set(ControlMode.PercentOutput, 0);
                           targetArmPosition = (Constants.ArmConstants.kMaxRotationUp - 1); 
                           status = "PASSED MAX POINT";
                    }
                else if (-operJoystick.getRawAxis(Constants.IOConstants.armAxisNum) < 0)
                   //if not passed min point
                   if(!(getArmPosition() <= Constants.ArmConstants.kMinRotationDown))
                   {
                                   // Set the arm motor to the new position based on the joystick
                          targetArmPosition = targetArmPosition + (-operJoystick.getRawAxis(Constants.IOConstants.armAxisNum) * Constants.ArmConstants.kArmSpeedMultiplier);
                          status = "moving down";
                        }
                   else
                    {
                        //STOP THAT MOTOR AAAA
                         //stop motor 
            rotationMotor.set(ControlMode.PercentOutput, 0);
            targetArmPosition = (Constants.ArmConstants.kMinRotationDown + 1); 
            status = "PASSED MIN POINT";
                    }
                }

            //set arm
            rotationMotor.set(ControlMode.Position, targetArmPosition);
        }).withName("idle (press trigger to take manual control)"));
}

public CommandBase setArmSpeed(double speed){
// Create a new command that will use the parallel command constructor.
return parallel(
    // Create a new command that will use the run command constructor.
    run(() -> {
        // Set the arm motor to the speed passed in
        targetArmPosition = targetArmPosition + (speed * Constants.ArmConstants.kArmSpeedMultiplier);
        // Update the shuffleboard
        updateShuffleboard();
    })).withName("setting arm speed");
}

public double getArmPosition(){
// Return the current position of the arm in ticks
return rotationMotor.getSelectedSensorPosition();
}

public double getArmVelocity(){
// Return the current velocity of the arm in ticks per ?ms
return rotationMotor.getSelectedSensorVelocity();
}

public void resetArmPosition(){
// Reset the arm position to 0
rotationMotor.setSelectedSensorPosition(0);
}

public void updateShuffleboard(){
// Update the shuffleboard, any values that are passed in prior to the update will be updated, even if they are not used in the update method.
Shuffleboard.update();
}

public CommandBase moveToBottomCommand(){
// Create a new command that will use the parallel command constructor.
return runOnce(() -> {
        // Set the arm motor to the bottom position
        targetArmPosition = Constants.ArmConstants.kBottomPosition;
        // Update the shuffleboard
        //updateShuffleboard();
    }).withName("move arm to bottom");
}

public CommandBase moveToMiddleCommand(){
// Create a new command that will use the parallel command constructor.
return runOnce(() -> {
        // Set the arm motor to the middle position
        targetArmPosition = Constants.ArmConstants.kMiddlePosition;
        // Update the shuffleboard
        //updateShuffleboard();
    }).withName("move arm to middle");
}

public CommandBase moveToTopCommand(){
// Create a new command that will use the parallel command constructor.
return runOnce(() -> {
        // Set the arm motor to the top position
        targetArmPosition = Constants.ArmConstants.kTopPosition;
        // Update the shuffleboard
        //updateShuffleboard();
    }).withName("move arm to top");
}


}
*/
