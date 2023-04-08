// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class AutoCommand extends CommandBase {
  private double starttime;
  private boolean completed;
  /** Creates a new AutoCommand. */
  public AutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    starttime = Timer.getFPGATimestamp();
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - starttime <= 0.2){
      Robot.pcmClaw.set(Value.kReverse);
    }
    else if(Timer.getFPGATimestamp() - starttime <= 3.35){
      Robot.m_VertLead.set(-0.25);
    }

    else if(Timer.getFPGATimestamp() - starttime <= 4.1){
      
      Robot.m_VertLead.set(-0.1);
    }
    else{
      completed = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return completed;
  }
}
