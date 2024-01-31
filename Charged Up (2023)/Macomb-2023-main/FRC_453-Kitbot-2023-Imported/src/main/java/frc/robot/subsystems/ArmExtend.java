



//the arm is dead.




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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ArmExtend extends SubsystemBase{
    private final WPI_TalonSRX extensionMotor = new WPI_TalonSRX(Constants.IOConstants.extensionSRXPort);
    public double targetArmPosition = 0;
    private Joystick operJoystick = new Joystick(Constants.IOConstants.kOperatorControllerPort);

    public ArmExtend(){
        // Set the arm motor to factory default settings
        extensionMotor.configFactoryDefault();
        
        // Set the brake mode
        extensionMotor.setNeutralMode(Constants.ArmConstants.kNeutralMode);
        
        // Set up the encoder
        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 1);
        extensionMotor.setSelectedSensorPosition(0);
        
        // Set the encoder phase
        extensionMotor.setSensorPhase(Constants.ArmConstants.kSensorPhase);
        
        // Set the direction of the motor
        extensionMotor.setInverted(Constants.ArmConstants.kMotorInvert);
        
        // Set the peak and nominal outputs. These are the values that the motor will be set to when the joystick is at its maximum value or normal value (Stopped)
        extensionMotor.configNominalOutputForward(0, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.configNominalOutputReverse(0, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.configPeakOutputForward(Constants.ArmConstants.kPeakOutput, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.configPeakOutputReverse(-Constants.ArmConstants.kPeakOutput, Constants.ArmConstants.kTimeoutMs);
        // Configure the closed loop error, which is the range of error where the motor output is held constant
        extensionMotor.configAllowableClosedloopError(Constants.ArmConstants.kPIDLoopIdx, 0, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.config_kF(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kF, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.config_kP(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kP, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.config_kI(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kI, Constants.ArmConstants.kTimeoutMs);
        extensionMotor.config_kD(Constants.ArmConstants.kPIDLoopIdx, Constants.ArmConstants.kD, Constants.ArmConstants.kTimeoutMs);

        // Setup double suppliers for pos and vel
        DoubleSupplier curArmVel = this::getArmVelocity;
        DoubleSupplier curArmPos = this::getArmPosition;

        // Set up the shuffleboard
        Shuffleboard.getTab("Arm").addNumber("Extension Position", curArmPos);
        Shuffleboard.getTab("Arm").addNumber("Extension Velocity", curArmVel);
        Shuffleboard.getTab("Arm").addNumber("Extension Target Position", () -> targetArmPosition);
        Shuffleboard.getTab("Arm").addNumber("Extension Error", () -> targetArmPosition - curArmPos.getAsDouble());
        Shuffleboard.getTab("Arm").add(this);

        // Set the default command for the arm. This should be the command that is run when no other commands are running

// Set the default command for the arm. This should be the command that is run when no other commands are running
setDefaultCommand(
    // Create a new command that will use the runOnce command constructor.
    run(
        () -> {
           //check if at max or min
           //if at maxPos and joystick is up, stop
              //if at minPos and joystick is down, stop
            if (getArmPosition() >= Constants.ArmConstants.kMaxPos || getArmPosition() <= Constants.ArmConstants.kMinPos){
                extensionMotor.set(ControlMode.PercentOutput, 0);
            
            } 
            else {
                //if running manual control, set to joystick speed
                if (operJoystick.getRawButton(Constants.IOConstants.kExtensionControlButton)){
                // Set the arm motor to the speed passed in
                extensionMotor.set(ControlMode.PercentOutput, operJoystick.getRawAxis(Constants.IOConstants.armAxisNum) * Constants.ArmConstants.kExtenderSpeed);
                }
                //else turn off
                else {
                    extensionMotor.set(ControlMode.PercentOutput, 0);
                }
            }
            //Hold the arm in place
            extensionMotor.set(ControlMode.Position, targetArmPosition);
        }).withName("extension control"));
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
    })).withName("set arm speed");
}

public double getArmPosition(){
// Return the current position of the arm in ticks
return extensionMotor.getSelectedSensorPosition();
}

public double getArmVelocity(){
// Return the current velocity of the arm in ticks per ?ms
return extensionMotor.getSelectedSensorVelocity();
}

public void resetArmPosition(){
// Reset the arm position to 0
extensionMotor.setSelectedSensorPosition(0);
}

public void updateShuffleboard(){
// Update the shuffleboard, any values that are passed in prior to the update will be updated, even if they are not used in the update method.
Shuffleboard.update();
}
}
*/