package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SwerveConstants;;

public final class HardwareConfigs {
  
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
    public SparkMaxConfig swerveAngleSparkConfig =  new SparkMaxConfig();
    public SparkMaxConfig swerveDriveSparkConfig =  new SparkMaxConfig();

    public HardwareConfigs(){
       /** Swerve CANCoder Configuration */
       swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;

       //Swerve angle motor config
       //Motor inverts and nuetral modes
       swerveAngleSparkConfig.inverted(SwerveConstants.angleMotorInvert);
       swerveAngleSparkConfig.idleMode(SwerveConstants.angleNuetralMode);

       //Gear ratio and wrapping config
       swerveAngleSparkConfig.encoder.positionConversionFactor(360/SwerveConstants.angleGearRatio);
       swerveAngleSparkConfig.encoder.velocityConversionFactor(SwerveConstants.angleGearRatio / 60);
       swerveAngleSparkConfig.closedLoop.positionWrappingEnabled(false);

       //current limiting
       swerveAngleSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(SwerveConstants.driveKP);
       swerveDriveSparkConfig.closedLoop.i(SwerveConstants.driveKI);
       swerveDriveSparkConfig.closedLoop.d(SwerveConstants.driveKD);

       //Swerve drive motor config
       //Motor inverts and nuetral modes
       swerveDriveSparkConfig.inverted(SwerveConstants.driveMotorInvert);
       swerveDriveSparkConfig.idleMode(SwerveConstants.driveNuetralMode);

       //Gear ratio and wrapping config
       swerveDriveSparkConfig.encoder.positionConversionFactor(SwerveConstants.wheelCircumference / SwerveConstants.driveGearRatio);
       swerveDriveSparkConfig.closedLoop.positionWrappingEnabled(true);

       //current limiting
       swerveDriveSparkConfig.smartCurrentLimit(40);

       //PID config
       swerveDriveSparkConfig.closedLoop.p(SwerveConstants.driveKP);
       swerveDriveSparkConfig.closedLoop.i(SwerveConstants.driveKI);
       swerveDriveSparkConfig.closedLoop.d(SwerveConstants.driveKD);

       swerveAngleSparkConfig.openLoopRampRate(SwerveConstants.openLoopRamp);
       swerveAngleSparkConfig.closedLoopRampRate(SwerveConstants.closedLoopRamp);

    }
}