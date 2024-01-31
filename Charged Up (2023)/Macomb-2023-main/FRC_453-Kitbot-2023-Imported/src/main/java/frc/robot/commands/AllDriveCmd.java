package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Robot;

public class AllDriveCmd extends CommandBase{
    //Create Objects

    //Create Constructor
    public AllDriveCmd(ExampleSubsystem subsystem){
        addRequirements(subsystem);
    }
    
    //Called when the command is initially scheduled.
    @Override
    public void initialize(){
        
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if(Robot.m_robotContainer.curDrive == false){
            
            Robot.m_robotContainer.allDrive = !Robot.m_robotContainer.allDrive;
        }
    }

    //Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        //Robot.m_robotContainer.m_pneumatics.defWheels();
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return true;
    }
}
