package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;

public class DefArmCmd extends CommandBase{
    //Create Objects
    private final PneumaticsSubsystem m_pneumaticsSubsystem;

    //Create Constructor
    public DefArmCmd(PneumaticsSubsystem subsystem){
        m_pneumaticsSubsystem = subsystem;
        addRequirements(m_pneumaticsSubsystem);
    }
    
    //Called when the command is initially scheduled.
    @Override
    public void initialize(){
        m_pneumaticsSubsystem.defArm();
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
