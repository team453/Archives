
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Robot;

public class OpenClawCmd extends CommandBase{
    //Create Objects

    //Create Constructor
    public OpenClawCmd(){

    }
    
    //Called when the command is initially scheduled.
    @Override
    public void initialize(){
        Robot.m_robotContainer.m_pneumatics.openClaw();
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
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
