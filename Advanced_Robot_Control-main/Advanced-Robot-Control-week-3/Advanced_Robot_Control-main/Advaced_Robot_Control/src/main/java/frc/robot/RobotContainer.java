// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //private final Shooter shooter = new Shooter();
  public static Joystick joystick1;
  public static Joystick joystick2;

  private final DriveToLine driveToLine;
  private final DriveTrain dt;
  private final MagicDrive magicDrive;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    joystick1 = new Joystick(0);
    joystick2 = new Joystick(Constants.USBOrder.One);

    dt = new DriveTrain();
    driveToLine = new DriveToLine(dt, 0.2);

    magicDrive = new MagicDrive(dt);


    // Configure the button bindings
    configureButtonBindings();
  }

  public static Joystick getJoy2() {
    return joystick2;
  }


 

  public static Joystick getJoy1() {
    return joystick1;
    }
    
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return magicDrive;
  }
  
}
