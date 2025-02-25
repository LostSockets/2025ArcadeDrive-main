package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
//Motors
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//WPI
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//FRC
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
    
    private final SparkMax rollerMotor = new SparkMax(Constants.RollerConstants.kRollerMotorPort, MotorType.kBrushed);
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

    public double getEncoderMeters() {
        return(((RelativeEncoder) rollerEncoder).getPosition());
    }

    public RollerSubsystem()
    {
    
    }

    public void setMotor(double speed) {
        SmartDashboard.putNumber("Roller Speed :", speed);
        rollerMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("RollerEncoder Value :", getEncoderMeters());
    }
}
