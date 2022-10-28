// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.subsystems.DriveTrain;


/** An example command that uses an example subsystem. */
public class MagicDrive  extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private double displacement = 1.0;
  private final DriveTrain dt;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MagicDrive(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    public MagicDrive(DriveTrain dt) {
      this.dt = dt;
    }   
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
  }
 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dt.magicDrive(displacement);
  }
 
  
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(dt.getDisplacement()) >= Math.abs(displacement);
  }
 
  
}
