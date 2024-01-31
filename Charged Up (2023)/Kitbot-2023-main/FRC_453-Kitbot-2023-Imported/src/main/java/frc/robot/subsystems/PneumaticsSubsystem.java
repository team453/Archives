package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticsSubsystem extends SubsystemBase{
    
    //Create new DoubleSolenoid objects
    private DoubleSolenoid m_solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

    public PneumaticsSubsystem (){
        
    }


    public void extendArm(){
        m_solenoid1.set(kForward);
    }

    public void retractArm(){
        m_solenoid1.set(kReverse);
    }

    public void defArm(){
        m_solenoid1.set(kOff);
    }
}
