package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.Constants;

public class ShooterCmd extends Command {
    
    private final RollerSubsystem rollerSubsystem;
    private final Supplier<Double> speedForward, speedReverse; 
    private final Supplier<Boolean> forwardShootButton, reverseShootButton;
    private final Joystick joyOperator = new Joystick(Constants.OIConstants.kOperatorJoystickPort);

    public ShooterCmd(RollerSubsystem rollerSubsystem, Supplier<Double> speedForward, Supplier<Double> speedReverse, Supplier<Boolean> forwardShootButton, Supplier<Boolean> reverseShootButton) {
        this.speedForward = speedForward;
        this.speedReverse = speedReverse;
        this.forwardShootButton = forwardShootButton;
        this.reverseShootButton = reverseShootButton;
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(rollerSubsystem);
    }

    @Override
    public void execute() {

        Boolean RunningLT = false;
        Boolean RunningRT = false;
        Boolean RunningB1 = false;
        Boolean RunningB2 = false;
        Boolean Running = false;

        //Trigger executions
        if (joyOperator.getRawAxis(Constants.OIConstants.kRollerForwardAxis) > 0.1) { //LT : Forward
            rollerSubsystem.setMotor(speedForward.get() * 0.5);
            RunningLT = true; } else {RunningLT = false;}
        if (joyOperator.getRawAxis(Constants.OIConstants.kRollerReverseAxis) > 0.1) { //RT : Reverse
            rollerSubsystem.setMotor(speedReverse.get() * 0.5);                          
            RunningRT = true; } else {RunningRT = false;}

        //Button executions
        if (joyOperator.getRawButton(Constants.OIConstants.kRollerForward) == true) { //Button : Forward
            rollerSubsystem.setMotor(Constants.RollerConstants.RUN_SPEED_FORWARD);
            RunningB1 = true; } else {RunningB1 = false;}
        if (joyOperator.getRawButton(Constants.OIConstants.kRollerReverse) == true) { //Button : Reverse
            rollerSubsystem.setMotor(Constants.RollerConstants.RUN_SPEED_REVERSE);
            RunningB2 = true; } else {RunningB2 = false;}

        if (RunningLT == true || RunningRT == true || RunningB1 == true || RunningB2 == true) {Running = true;} else {Running = false;}
        
        if (Running != true) {rollerSubsystem.setMotor(0);}
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
