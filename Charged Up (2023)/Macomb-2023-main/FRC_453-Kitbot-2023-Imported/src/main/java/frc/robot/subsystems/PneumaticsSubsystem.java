package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class PneumaticsSubsystem extends SubsystemBase{
    
    //Create new DoubleSolenoid objects
    public DoubleSolenoid m_solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.kForwardChannel, ArmConstants.kReverseChannel);
    public DoubleSolenoid m_tankSolLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, DriveConstants.ktankSolLeftOut, DriveConstants.ktankSolLeftIn);
   
    //AnalogInput analog = new AnalogInput(0);

    public PneumaticsSubsystem (){
        
    }

    public void closeClaw(){
        
        m_solenoid1.set(kForward);
    }

    public void openClaw(){
        m_solenoid1.set(kReverse);
    }

    public void defArm(){
        m_solenoid1.set(kOff);
    }

    public void wheelsUp(){
        m_tankSolLeft.set(kForward);
    }

    public void wheelsDown(){
        m_tankSolLeft.set(kReverse);
    }

    public void defWheels(){
        m_tankSolLeft.set(kOff);
    }
}
