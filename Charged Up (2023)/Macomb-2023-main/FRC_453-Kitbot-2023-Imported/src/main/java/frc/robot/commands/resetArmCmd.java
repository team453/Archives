
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Constants;

import frc.robot.Robot;

public class resetArmCmd extends CommandBase{
    //Create Objects

    //Create Constructor
    public resetArmCmd(){

    }
    
    //Called when the command is initially scheduled.
    @Override
    public void initialize(){
       
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
      //  Robot.m_robotContainer.m_arm.resetArmPosition();
    }

    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return true;
    }
}
