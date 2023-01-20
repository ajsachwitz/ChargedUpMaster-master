package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBase extends SubsystemBase {
  //check with Mr. McCarthy if any of the Motors or encoders are reversed
  public final SwerveModule frontLeft = new SwerveModule(
    Swerve.frontLeftDriveMotorId, 
    Swerve.frontLeftRotationMotorId,
    true,
    true, 
    Swerve.frontLeftRotationEncoderId, 
    Swerve.frontLeftAbsoluteEncoderOffset, 
    false);

  public final SwerveModule frontRight = new SwerveModule(
    Swerve.frontRightDriveMotorId, 
    Swerve.frontRightRotationMotorId,
    true,
    true, 
    Swerve.frontRightRotationEncoderId, 
    Swerve.frontRightAbsoluteEncoderOffset, 
    false);

  public final SwerveModule rearLeft = new SwerveModule(
    Swerve.rearLeftDriveMotorId, 
    Swerve.rearLeftRotationMotorId,
    true,
    true, 
    Swerve.rearLeftRotationEncoderId, 
    Swerve.rearLeftAbsoluteEncoderOffset, 
    false);

  public final SwerveModule rearRight = new SwerveModule(
    Swerve.rearRightDriveMotorId, 
    Swerve.rearRightRotationMotorId,
    true,
    true, 
    Swerve.rearRightRotationEncoderId, 
    Swerve.rearRightAbsoluteEncoderOffset, 
    false);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  public SwerveBase(){
    //waits second to reset Gyroscope since it is rebooting
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e){
      }
    }).start();
    
    
  }

  public void zeroHeading() {
    gyro.reset();
  }

  //puts gyroscope between -180,180 degrees
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(),360);
  }

  //calls getHeating and puts it into a rotation2d from a degree
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }
  
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    rearLeft.stop();
    rearRight.stop();
  }

  public void setModuleState(SwerveModuleState[] desiredStates){
    //maxes desired states at the max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Swerve.maxSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

}
