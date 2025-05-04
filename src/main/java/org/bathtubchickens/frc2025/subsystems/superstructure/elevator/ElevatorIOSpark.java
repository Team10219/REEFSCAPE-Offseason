// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.subsystems.superstructure.elevator;

import static org.bathtubchickens.frc2025.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class ElevatorIOSpark implements ElevatorIO {
    public static final double reduction = 9.0;

    private final SparkMax spark;
    private final SparkMax followerSpark;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig config;

    private final Debouncer connectedDebouncer = new Debouncer(0.5);
    private int currentLimit = 80;
    private boolean brakeModeEnabled = true;

    public ElevatorIOSpark(int deviceId) {
        spark = new SparkMax(deviceId, MotorType.kBrushless);
        followerSpark = new SparkMax(deviceId, MotorType.kBrushless);
        encoder = spark.getEncoder();

        config = new SparkMaxConfig();
        config 
            .idleMode(
                brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(currentLimit, 50)
            .voltageCompensation(12.0);
        config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
        config 
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            spark,
            5,
            () ->
                spark.configure(
                    config,
                    SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters));
        tryUntilOk(spark, 5, () -> encoder.setPosition(0.0));
        tryUntilOk(
            followerSpark, 
            5,
            () ->
                followerSpark.configure(
                    config.follow(spark.getDeviceId(), true), 
                    SparkBase.ResetMode.kResetSafeParameters,
                    SparkBase.PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sparkStickyFault = false;
        inputs.data =
            new ElevatorIOData(
                ifOkOrDefault(spark, encoder::getPosition, inputs.data.positionRads()),
                ifOkOrDefault(spark, encoder::getVelocity, inputs.data.velocityRadsPerSec()),
                ifOkOrDefault(
                    spark,
                    new DoubleSupplier[] {spark::getBusVoltage, spark::getAppliedOutput},
                    x -> x[0] * x[1],
                    inputs.data.appliedVoltage()),
                0.0,
                ifOkOrDefault(spark, spark::getOutputCurrent, inputs.data.torqueCurrentAmps()),
                ifOkOrDefault(spark, spark::getMotorTemperature, inputs.data.tempCelsius()),
                ifOkOrDefault(
                    followerSpark,
                    new DoubleSupplier[] {followerSpark::getBusVoltage, spark::getAppliedOutput},
                    x -> x[0] * x[1],
                    inputs.data.followerAppliedVoltage()),
                0.0,
                ifOkOrDefault(followerSpark, followerSpark::getOutputCurrent, inputs.data.followerTorqueCurrentAmps()),
                ifOkOrDefault(followerSpark, followerSpark::getMotorTemperature, inputs.data.followerTempCelsius()),
                false,
                false,
                connectedDebouncer.calculate(!sparkStickyFault),
                connectedDebouncer.calculate(!sparkStickyFault));
    }

    @Override
    public void runVolts(double volts) {
        spark.set(volts);
    }

    @Override
    public void stop() {
        spark.stopMotor();
    }

    @Override 
    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        new Thread(
            () ->
                tryUntilOk(
                    spark,
                    5, 
                    () ->
                        spark.configure(
                            config.idleMode(
                                brakeModeEnabled
                                ? SparkBaseConfig.IdleMode.kBrake
                                : SparkBaseConfig.IdleMode.kCoast), 
                            SparkBase.ResetMode.kResetSafeParameters, 
                            SparkBase.PersistMode.kPersistParameters)))
        .start();
    }
}


