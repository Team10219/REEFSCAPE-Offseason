// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public ElevatorIOData data =
            new ElevatorIOData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false, false, false);
    }

    record ElevatorIOData(
        double positionRads,
        double velocityRadsPerSec,
        double appliedVoltage,
        double torqueCurrentAmps,
        double supplyCurrentAmps,
        double tempCelsius,
        double followerAppliedVoltage,
        double followerTorqueCurrentAmps,
        double followerSupplyCurrentAmps,
        double followerTempCelsius,
        boolean tempFault,
        boolean followerTempFault,
        boolean motorConnected,
        boolean followerConnected
        ) {}

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runOpenLoop(double output) {}

    default void runVolts(double volts) {}

    default void stop() {}

    default void runPosition(double positionRad, double feedforward) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setBrakeMode(boolean enabled) {}
 }
