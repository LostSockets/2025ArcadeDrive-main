
package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJoyCmd extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> speedFunction;


    double realTimeSpeed = 0;

   
    public ElevatorJoyCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Double> speedFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speedFunction = speedFunction;
        addRequirements(elevatorSubsystem);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (Math.abs(realTimeSpeed) < 0.05) {
      realTimeSpeed = 0;
    }

//    if (elevatorSubsystem.getEncoderMeters() < Constants.ElevatorConstants.kmaxElevatorEncoderHeight && elevatorSubsystem.getEncoderMeters() > Constants.ElevatorConstants.kminElevatorEncoderHeight) {
//      realTimeSpeed = speedFunction.get() * Constants.ElevatorConstants.kElevatorSpeedPercentage;
//      elevatorSubsystem.setMotor(-realTimeSpeed);
    realTimeSpeed = speedFunction.get() * Constants.ElevatorConstants.kElevatorSpeedPercentage;

    if (elevatorSubsystem.getEncoderMeters() < Constants.ElevatorConstants.kmaxElevatorEncoderHeight && realTimeSpeed > 0.0)
      elevatorSubsystem.setMotor(-realTimeSpeed);
    if (elevatorSubsystem.getEncoderMeters() > Constants.ElevatorConstants.kminElevatorEncoderHeight && realTimeSpeed < 0.0)
      elevatorSubsystem.setMotor(-realTimeSpeed);
  
    //System.out.println("speed = " + realTimeSpeed);
    //System.out.println("encoder = " + ElevatorSubsystem.getEncoderMeters());
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}