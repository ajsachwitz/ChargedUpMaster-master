package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

public class SwerveModule{
  //sparkMax encoders
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;
  //motor encoders
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;
  //pid controller to move angle motor
  private final PIDController turningPidController;
  //absolute encoder, if reversed and offset position
  private final AnalogInput absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
                      int absoluteEncoderID, double AbsoluteEncoderOffset, boolean absoluteEncoderReversed){
      this.absoluteEncoderOffsetRad = AbsoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new AnalogInput(absoluteEncoderID);

      driveMotor = new CANSparkMax(driveMotorID,MotorType.kBrushless);
      turningMotor = new CANSparkMax(turningMotorID,MotorType.kBrushless);

      driveMotor.setInverted(driveMotorReversed);
      turningMotor.setInverted(turningMotorReversed);

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = turningMotor.getEncoder();

      driveEncoder.setPositionConversionFactor(2.0 * Math.PI / Swerve.driveGearRatio);
      driveEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / Swerve.driveGearRatio);
      turningEncoder.setPositionConversionFactor(2.0 * Math.PI / Swerve.angleGearRatio);
      turningEncoder.setVelocityConversionFactor(2.0 * Math.PI / 60 / Swerve.angleGearRatio);

      turningPidController = new PIDController(0.5, 0, 0.0);
      turningPidController.enableContinuousInput(-Math.PI,Math.PI);    

      resetEncoders();
  }

  public double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  public double getTurningPosition(){
    return driveEncoder.getPosition();
  }

  //might have to change these
  public double getDriveVelocity(){
    return driveEncoder.getVelocity();
  }

  public double getTurngingVelocity(){
    return driveEncoder.getVelocity ();
  }


  public double getAbsoluteEncoderRad() {
    //gives percent of a full rotation
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *=2.0 * Math.PI;
    angle -=absoluteEncoderOffsetRad;
    //checks if reversed, if so multiply by -1
    return angle * (absoluteEncoderReversed ? -1.0:1.0);
  }

  public void resetEncoders(){
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());

  }
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state){
    if(Math.abs(state.speedMetersPerSecond)< 0.001){
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Swerve.maxSpeed);
    turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + " ]",state.toString());
    
  }
  public void stop(){
    driveMotor.set(0);
    turningMotor.set(0);
  }
}