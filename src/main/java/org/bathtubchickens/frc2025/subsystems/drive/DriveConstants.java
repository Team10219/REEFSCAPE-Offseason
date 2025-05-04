// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/** Add your docs here. */
public class DriveConstants {
    private static final Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = 
        new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final DriveMotorArrangement kDriveMotorType = 
        DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType =
        SteerMotorArrangement.TalonFX_Integrated;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    private static final Current kSlipCurrent = Amps.of(120);

    private static final TalonFXConfiguration driveInitalConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true));
    private static final TalonFXConfiguration steerInitialConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))); // might be worth to change to 50 amps cause steer doesn't need that much
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final CANBus kCANBus = new CANBus("Tim", "./logs/example.hoot");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(10.0);

    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 6.75;
    private static final double kSteerGearRatio = 25;
    private static final Distance kWheelRadius = Inches.of(1.7);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 10;

    // Only used for SIM
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
    // Sim Voltage
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants =
        new SwerveDrivetrainConstants()
        .withCANBusName(kCANBus.getName())
        .withPigeon2Id(kPigeonId)
        .withPigeon2Configs(pigeonConfigs);

    public static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator = 
            new SwerveModuleConstantsFactory<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitalConfig)
            .withSteerMotorInitialConfigs(steerInitialConfig)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 43;
    private static final int kFrontLeftSteerMotorId = 42;
    private static final int kFrontLeftEncoderId = 41;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.40234375);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(12.5);
    private static final Distance kFrontLeftYPos = Inches.of(12.5);

    // Front Right
    private static final int kFrontRightDriveMotorId = 33;
    private static final int kFrontRightSteerMotorId = 42;
    private static final int kFrontRightEncoderId = 41;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.011962890625);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(12.5);
    private static final Distance kFrontRightYPos = Inches.of(-12.5);

    // Back Left
    private static final int kBackLeftDriveMotorId = 13;
    private static final int kBackLeftSteerMotorId = 12;
    private static final int kBackLeftEncoderId = 11;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.30908203125);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-12.5);
    private static final Distance kBackLeftYPos = Inches.of(12.5);

    // Back Right
    private static final int kBackRightDriveMotorId = 23;
    private static final int kBackRightSteerMotorId = 22;
    private static final int kBackRightEncoderId = 21;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.17333984375);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-12.5);
    private static final Distance kBackRightYPos = Inches.of(-12.5);

    public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new , drivetrainConstants, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}