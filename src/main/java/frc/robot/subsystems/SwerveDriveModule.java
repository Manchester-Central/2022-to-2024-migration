// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.chaos131.pid.PIDFValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class SwerveDriveModule {
    private TalonFXConfiguration m_velocityConfig = new TalonFXConfiguration();
    private TalonFXConfiguration m_angleConfig = new TalonFXConfiguration();
    private VelocityVoltage m_velocityVoltageMps = new VelocityVoltage(0);
    private PositionVoltage m_positionVoltageRotations = new PositionVoltage(0);
    private Translation2d m_location;
    private Field2d m_field = new Field2d();
    private double m_targetVelocity = 0;
    private double m_targetAngleDegrees = 0;
    private TalonFX m_velocityController;
    private TalonFX m_angleController;
    private String m_name;
    private CANcoder m_absoluteEncoder;
    private double m_simDistance = 0;


    public SwerveDriveModule(
        double x, 
        double y, 
        String name, 
        int velocityControllerPort,
        int angleControllerPort, 
        int absoluteEncoderPort, 
        Rotation2d absoluteAngleOffset,
        InvertedValue velocityMotorDirection
    ) {
        m_location = new Translation2d(x, y);
        if (RobotBase.isSimulation()) {
            SmartDashboard.putData(name, m_field);
        }
        m_velocityController = new TalonFX(velocityControllerPort);
        m_angleController = new TalonFX(angleControllerPort);
        m_absoluteEncoder = new CANcoder(absoluteEncoderPort);

        m_velocityConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_velocityConfig.MotorOutput.Inverted = velocityMotorDirection;
        // m_velocityConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // m_velocityConfig.Feedback.SensorToMechanismRatio = Constants.ModuleVelocityRotorToSensorRatio;
        // m_velocityController.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_20Ms);
        // m_velocityController.configVelocityMeasurementWindow(32);
        m_velocityController.getConfigurator().apply(m_velocityConfig);

        m_angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        m_angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        m_angleConfig.Feedback.SensorToMechanismRatio = Constants.SwerveModuleAngleGearRatio;
        //m_angleConfig.ClosedLoopGeneral.ContinuousWrap = true;
        // m_angleController.configAllowableClosedloopError(0, DegreesToFalconAngle(0.5)); //TODO Reduce after tuning PID
        m_angleController.getConfigurator().apply(m_angleConfig);

        // m_angleController.setPosition(0);
        
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = absoluteAngleOffset.getRotations();
         m_absoluteEncoder.getConfigurator().apply(canCoderConfig);
         m_absoluteEncoder.getAbsolutePosition().setUpdateFrequency(4);

        teleopInit();
        m_name = name;
        
        var absoluteEncoderAngle = GetAbsoluteEncoderAngle();
        //var angleTicksOffset = this.DegreesToFalconAngle(absoluteEncoderAngle);
         m_angleController.setPosition(absoluteEncoderAngle.getRotations());
        Robot.LogManager.addNumber(m_name + "/targetVelocityMPS", () -> m_targetVelocity);
        Robot.LogManager.addNumber(m_name + "/targetAngleDegrees", () -> m_targetAngleDegrees);
        Robot.LogManager.addNumber(m_name + "/actualVelocityMPS", () -> getCurrentVelocityMPS());
        Robot.LogManager.addNumber(m_name + "/actualAngleDegrees", () -> getCurrentAngle().getDegrees());
        Robot.LogManager.addNumber(m_name + "/AbsoluteAngleDegrees", () -> GetAbsoluteEncoderAngle().getDegrees());
        Robot.LogManager.addNumber(m_name + "/DriveShaftSpeed", () -> m_velocityController.getVelocity().getValueAsDouble());
        Robot.LogManager.addNumber(m_name + "/rotorPosition", () -> m_angleController.getPosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition() {

        if (Robot.isSimulation()) {
            m_simDistance = m_simDistance + m_targetVelocity / Constants.RobotUpdate_hz;
            return new SwerveModulePosition(m_simDistance, Rotation2d.fromDegrees(m_targetAngleDegrees));
        }

        double distance = m_velocityController.getPosition().getValueAsDouble();
        Rotation2d angle = getCurrentAngle();
        distance = FalconTicksToMeters(distance);
        return new SwerveModulePosition(distance,angle);
    }

    public void autoInit() {
        m_velocityConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.65;
        m_velocityController.getConfigurator().apply(m_velocityConfig);
    }

    public void teleopInit() {
        m_angleConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
        m_angleController.getConfigurator().apply(m_angleConfig);
    }

    public void updatePosition(Pose2d robotPose) {
        Pose2d modulePose = robotPose.transformBy(new Transform2d(m_location, Rotation2d.fromDegrees(m_targetAngleDegrees)));
        m_field.setRobotPose(modulePose);
    }

    public double getCurrentVelocityMPS() {
        if (RobotBase.isReal()) {
            return FalconVelocityToMPS(m_velocityController.getVelocity().getValueAsDouble());
        }
        return m_targetVelocity;
    }

    public Rotation2d getCurrentAngle() {
        if (RobotBase.isReal()) {
            return Rotation2d.fromRotations(m_angleController.getPosition().getValueAsDouble());
        }
        return Rotation2d.fromDegrees(m_targetAngleDegrees);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getCurrentVelocityMPS(),getCurrentAngle().times(-1));
    }

    public void setTargetState(SwerveModuleState targetState) {
        targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        m_targetVelocity = targetState.speedMetersPerSecond;
        m_targetAngleDegrees = AngleUtil.closestTarget(getCurrentAngle().getDegrees(), targetState.angle.getDegrees());

        // m_velocityController.set(0.3);
        // m_angleController.set(0.3);
        m_velocityVoltageMps.Slot = 0;
        m_velocityController.setControl(m_velocityVoltageMps.withVelocity(m_targetVelocity / Constants.ModuleVelocityRotorToSensorRatio));
        m_positionVoltageRotations.Slot = 0;
        m_angleController.setControl(m_positionVoltageRotations.withPosition(Rotation2d.fromDegrees(m_targetAngleDegrees).getRotations()));
        //m_angleController.setControl(m_positionVoltageRotations.withPosition(Rotation2d.fromDegrees(m_absoluteAngleOffset).getRotations()));
    }

    public void setManual(double velocityControllerPower, double angleControllerPower) {
        m_velocityController.set(velocityControllerPower);
        m_angleController.set(angleControllerPower);
    }

    public Translation2d getLocation() {
        return m_location;
    }

    private double MPSToFalconVelocity(double mps) {
        // Distance conversion from meters to encoder counts

        var rps = mps / Constants.DriveWheelCircumferenceMeters;
        var countsPerSecond = rps * Constants.SwerveModuleVelocityGearRatio * Constants.TalonCountsPerRevolution;

        // Time conversion from seconds to 100 milliseconds

        return countsPerSecond / 10;

    }

    private double FalconVelocityToMPS(double FalconVelocity) {
        var countsPerSecond = FalconVelocity * 10;
        var rps = countsPerSecond / (Constants.SwerveModuleVelocityGearRatio * Constants.TalonCountsPerRevolution);
        var mps = rps * Constants.DriveWheelCircumferenceMeters;
        return mps;
    }

    private double FalconTicksToMeters(double counts) {
        var revolutions = counts / Constants.TalonCountsPerRevolution;
        var wheelRotations = revolutions / Constants.SwerveModuleVelocityGearRatio;
        var meters = wheelRotations * Constants.DriveWheelCircumferenceMeters;
        return meters;
    }



    public void UpdateVelocityPIDConstants(PIDFValue update) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = update.P;
        slot0Configs.kI = update.I;
        slot0Configs.kD = update.D;
        slot0Configs.kV = update.F;
        slot0Configs.kS = 0.05;

        m_velocityController.getConfigurator().apply(slot0Configs);
    }

    public void UpdateAnglePIDConstants(PIDFValue update) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = update.P;
        slot0Configs.kI = update.I;
        slot0Configs.kD = update.D;
        slot0Configs.kV = update.F;
        m_angleController.getConfigurator().apply(slot0Configs);
    }

    public void ResetEncoders() {
        m_velocityController.setPosition(0);
        m_angleController.setPosition(0);
    }

    public void Stop() {
        // Update these values to fix display issues in the simulator
        m_targetVelocity = 0;
        m_targetAngleDegrees = getCurrentAngle().getDegrees();

        setManual(0, 0);
    }

    private Rotation2d GetAbsoluteEncoderAngle() {
        return Rotation2d.fromRotations( m_absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        // if (RobotBase.isReal()) {
        //     return 360 - m_absoluteEncoder.getAbsolutePosition().getValueAsDouble() + m_absoluteAngleOffset;
        // }
        // return m_targetAngleDegrees;

    }



}