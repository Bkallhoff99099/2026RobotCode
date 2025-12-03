// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ModuleConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;



public class SwerveModule {

    private TalonFXS driveMotor;
    private TalonFXS steerMotor;
    private CANcoder    absoluteEncoder;
    //private SparkMaxPIDController drivingPIDController;
    //private SparkMaxPIDController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new TalonFXS(driveMotorCANID);
        steerMotor = new TalonFXS(steerMotorCANID);
        TalonFXSConfigurator driveConfigurator = driveMotor.getConfigurator();
        TalonFXSConfiguration driveConfig = new TalonFXSConfiguration();
        TalonFXSConfigurator steerConfigurator = steerMotor.getConfigurator();
        TalonFXSConfiguration steerConfig = new TalonFXSConfiguration();
        absoluteEncoder = new CANcoder(cancoderCANID);
        
        // Get the PID Controllers
        //drivingPIDController = driveMotor.getPIDController();
        //turningPIDController = steerMotor.getPIDController();

        var slot0ConfigsDrive = driveConfig.Slot0;
        var slot0ConfigsSteer = steerConfig.Slot0;
        
        // Get the encoders
        //driveEncoder = driveMotor.getEncoder();
        //steerEncoder = steerMotor.getEncoder();
        
        // Reset everything to factory default
        //driveMotor.restoreFactoryDefaults();
        //steerMotor.restoreFactoryDefaults();
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
        // Continue configuration here..
        
        // CANcoder Configuration
        CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        
        cfg.apply(magnetSensorConfiguration
                    //AbsoluetSensorDiscontinuity point is new for the 2026 season
                    //Value 0.5 that was input may be wrong
                  .withAbsoluteSensorDiscontinuityPoint(.5)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        // Steering Motor Configuration
        //steerMotor.setInverted(false);
        //turningPIDController.setFeedbackDevice(steerEncoder);

        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        //steerEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        //steerEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        //turningPIDController.setPositionPIDWrappingEnabled(true);
        //turningPIDController.setPositionPIDWrappingMinInput(0);
        //turningPIDController.setPositionPIDWrappingMaxInput(90);
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        //turningPIDController.setP(ModuleConstants.kTurningP);
        //turningPIDController.setI(ModuleConstants.kTurningI);
        //turningPIDController.setD(ModuleConstants.kTurningD);
        //turningPIDController.setFF(ModuleConstants.kTurningFF);
        slot0ConfigsSteer.kP = ModuleConstants.kTurningP;
        slot0ConfigsSteer.kI = ModuleConstants.kTurningI;
        slot0ConfigsSteer.kD = ModuleConstants.kTurningD;
        slot0ConfigsSteer.kG = ModuleConstants.kTurningFF;
        





        // Drive Motor Configuration
        //driveMotor.setInverted(false);
        //drivingPIDController.setFeedbackDevice(driveEncoder);
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.        
        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!

        
        //drivingPIDController.setP(ModuleConstants.kDrivingP);
        //drivingPIDController.setI(ModuleConstants.kDrivingI);
        //drivingPIDController.setD(ModuleConstants.kDrivingD);
        //drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        slot0ConfigsDrive.kP = ModuleConstants.kDrivingP;
        slot0ConfigsDrive.kI = ModuleConstants.kDrivingI;
        slot0ConfigsDrive.kD = ModuleConstants.kDrivingD;
        slot0ConfigsDrive.kG = ModuleConstants.kDrivingFF;


        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        //driveMotor.burnFlash();
        //steerMotor.burnFlash();
        driveConfigurator.apply(driveConfig);
        steerConfigurator.apply(steerConfig);
        
        
        //driveEncoder.setPosition(0);
        //steerEncoder.setPosition(encoder.getAbsolutePosition().refresh().getValue() * 360);
        driveMotor.setPosition(0);
        steerMotor.setPosition(absoluteEncoder.getAbsolutePosition().refresh().getValue());
    }
    
    
    /**
    Get the distance in meters.
    */
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }
    
    /**
    Get the angle.
    */
    public Rotation2d getAngle()
    {
          return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    */
    public void setState(SwerveModuleState state)
    {
        //turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        //drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        PositionDutyCycle steerControl = new PositionDutyCycle(state.angle.getDegrees());
        VelocityDutyCycle driveControl = new VelocityDutyCycle(state.speedMetersPerSecond);
        steerMotor.setControl(steerControl);
        driveMotor.setControl(driveControl);


          
    }

}