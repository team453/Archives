package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.Robot;

public class SwapDriveCmd extends CommandBase{
    //Create Objects

    //Create Constructor
    public SwapDriveCmd(ExampleSubsystem subsystem){
        addRequirements(subsystem);
    }
    
    //Called when the command is initially scheduled.
    @Override
    public void initialize(){
        
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        Robot.m_robotContainer.curDrive = !Robot.m_robotContainer.curDrive;
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
