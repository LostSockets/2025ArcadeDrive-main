// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Constants
//import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.ArmTelescopicConstants;

//Commands
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.ArcadeDrivePIDHoldInPlaceCmd;
import frc.robot.commands.AutoDelay;
import frc.robot.commands.AutoDriveBkwdCmd;
import frc.robot.commands.AutoDriveFwdCmd;
//import frc.robot.commands.ElevatorButtonCmd;
import frc.robot.commands.ElevatorJoyCmd;
import frc.robot.commands.ElevatorPIDCmd;
import frc.robot.commands.ShooterCmd;
//import frc.robot.commands.ArmPivotButtonCmd;


//Subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RollerSubsystem;


//Libraries
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final Joystick joyDrive = new Joystick(Constants.OIConstants.kDriverJoystickPort);
  private final Joystick joyOperator = new Joystick(Constants.OIConstants.kOperatorJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Default commands
    driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(driveSubsystem, () -> -joyDrive.getRawAxis(Constants.OIConstants.kArcadeDriveSpeedAxis), () -> -joyDrive.getRawAxis(Constants.OIConstants.kArcadeDriveTurnAxis)));
    //elevatorSubsystem.setDefaultCommand(new ElevatorButtonCmd(elevatorSubsystem, () -> -joyOperator.getRawAxis(Constants.OIConstants.kElevatorAxis)));
    elevatorSubsystem.setDefaultCommand(new ElevatorJoyCmd(elevatorSubsystem, () -> -joyOperator.getRawAxis(Constants.OIConstants.kElevatorAxis)));

    configureBindings();
    rollerSubsystem.setDefaultCommand(new ShooterCmd(rollerSubsystem,
                                                    () -> joyOperator.getRawAxis(Constants.OIConstants.kRollerForwardAxis),
                                                    () -> -joyOperator.getRawAxis(Constants.OIConstants.kRollerReverseAxis),
                                                    () -> joyOperator.getRawButton(Constants.OIConstants.kRollerForward),
                                                    () -> joyOperator.getRawButton(Constants.OIConstants.kRollerReverse)));

  }

  private void configureBindings() {
    new JoystickButton(joyOperator, Constants.OIConstants.kElevatorPIDPos0Button).onTrue(new ElevatorPIDCmd(elevatorSubsystem, Constants.ElevatorConstants.kElevatorPIDPos0));
    new JoystickButton(joyOperator, Constants.OIConstants.kElevatorPIDPos1Button).onTrue(new ElevatorPIDCmd(elevatorSubsystem, Constants.ElevatorConstants.kElevatorPIDPos1));
    new JoystickButton(joyOperator, Constants.OIConstants.kElevatorPIDPos2Button).onTrue(new ElevatorPIDCmd(elevatorSubsystem, Constants.ElevatorConstants.kElevatorPIDPos2));
    new JoystickButton(joyOperator, Constants.OIConstants.kElevatorPIDPos3Button).onTrue(new ElevatorPIDCmd(elevatorSubsystem, Constants.ElevatorConstants.kElevatorPIDPos3));
    //armPivotSubsystem.setDefaultCommand(new ArmPivotJoyCmd(armPivotSubsystem, () -> -joyArm.getRawAxis(Constants.OIConstants.kArmPivotAxis)));
    //new JoystickButton(joyArm, Constants.OIConstants.kArmPivotDown).whileTrue(new ArmPivotButtonCmd(armPivotSubsystem, Constants.ArmPivotConstants.kArmPivotSpeedPercentageThrottled));
    //new JoystickButton(joyArm, Constants.OIConstants.kArmPivotUp).whileTrue(new ArmPivotButtonCmd(armPivotSubsystem, -1*Constants.ArmPivotConstants.kArmPivotSpeedPercentageThrottled));
    new JoystickButton(joyDrive, Constants.OIConstants.kArcadeDriveHoldInPlace).whileTrue((new ArcadeDrivePIDHoldInPlaceCmd(driveSubsystem)));
  }

  public Command getAutonomousCommand() {

    /* drive forward and wait */
    return new SequentialCommandGroup(
      new AutoDriveFwdCmd(driveSubsystem, Constants.AutoConstants.kAutoDriveFwdDistance),
      new AutoDelay(10)
       //new AutoPivot(armPivotSubsystem, Constants.AutoConstants.kAutoPivotHeight2),
      //new AutoDriveBkwdCmd(driveSubsystem, Constants.AutoConstants.kAutoDriveBkwdDistance)
     

    
    /*
    //  Autonomous X:  TESTING

    return 
    new SequentialCommandGroup(
      new AutoPivot(armPivotSubsystem, Constants.AutoConstants.kAutoPivotHeight1),
      new ParallelCommandGroup(
        new AutoPivot(armPivotSubsystem, Constants.AutoConstants.kAutoPivotHeight1),
        new AutoDriveFwdCmd(driveSubsystem, 10)
      ),
      new AutoDelay(2),
      new AutoDriveBkwdCmd(driveSubsystem, 2)
    */

    );//
  }
}
