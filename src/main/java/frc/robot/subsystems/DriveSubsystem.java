package frc.robot.subsystems;

//import com.revrobotics.CANEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.cameraserver.CameraServer;

public class DriveSubsystem extends SubsystemBase {

  // drivetrain motors
  private final SparkMax driveTrainLM1 = new SparkMax(Constants.DriveConstants.kLeftMotor1Port, MotorType.kBrushed);
  private final SparkMax driveTrainLM2 = new SparkMax(Constants.DriveConstants.kLeftMotor2Port, MotorType.kBrushed);
  private final SparkMax driveTrainRM3 = new SparkMax(Constants.DriveConstants.kRightMotor1Port, MotorType.kBrushed);
  private final SparkMax driveTrainRM4 = new SparkMax(Constants.DriveConstants.kRightMotor2Port, MotorType.kBrushed);

  private final RelativeEncoder driveLM1Encoder = driveTrainLM1.getEncoder();
  private final RelativeEncoder driveRM3Encoder = driveTrainRM3.getEncoder();

  public double getEncoderMeters() {
    return ((((RelativeEncoder) driveLM1Encoder).getPosition() + -((RelativeEncoder) driveRM3Encoder).getPosition()) / 2) * Constants.DriveConstants.kEncoderTick2Feet;
  }
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    CameraServer.startAutomaticCapture();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          //run once
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("DriveEncoder Value",  getEncoderMeters());
  }

  public void setMotors(double leftSpeed, double rightSpeed) {
    driveTrainLM1.set(-leftSpeed);
    driveTrainLM2.set(leftSpeed);
    driveTrainRM3.set(rightSpeed);
    driveTrainRM4.set(rightSpeed);
    SmartDashboard.putNumber("leftSpeed: ", leftSpeed);
    SmartDashboard.putNumber("RightSpeed: ", rightSpeed);
  }

}
