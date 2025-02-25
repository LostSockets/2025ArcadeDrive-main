package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPIDCmd extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final PIDController pidController;
    private final Joystick joyArm = new Joystick(Constants.OIConstants.kArmJoystickPort);

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(Constants.ArmPivotConstants.kPButton,Constants.ArmPivotConstants.kIButton,Constants.ArmPivotConstants.kDButton);
        pidController.setSetpoint(setpoint);
        addRequirements(elevatorSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    double speed = pidController.calculate(elevatorSubsystem.getEncoderMeters());
    elevatorSubsystem.setMotor(speed);
    System.out.println("Arm speed = " + speed);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(joyArm.getRawAxis(Constants.OIConstants.kArmPivotAxis)) > 0.05) {
      return true;
    } else {
      return false;
    }
  }
}
