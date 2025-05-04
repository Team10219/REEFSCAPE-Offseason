// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.bathtubchickens.frc2025.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

@AutoLog
public interface ModuleIO {
    class ModuleIOInputs {
        public ModuleIOData data =
            new ModuleIOData(false, 0.0, 0.0, 0.0, 0.0, false, false, new Rotation2d(), new Rotation2d(), 0.0, 0.0, 0.0, new double[] {}, new double[] {}, new Rotation2d[] {});

    }

    record ModuleIOData(
        boolean driveConnected,
        double drivePositionRad,
        double driveVelocityRadPerSec,
        double driveAppliedVolts,
        double driveCurrentAmps,

        boolean turnConnected,
        boolean turnEncoderConnected,
        Rotation2d turnAbsolutePosition,
        Rotation2d turnPosition,
        double turnVelocityRadPerSec,
        double turnAppliedVolts,
        double turnCurrentAmps,

        double[] odometryTimestamps,
        double[] odometryDrivePositionsRad,
        Rotation2d[] odometryTurnPositions
    ) {}

    default void updateInputs(ModuleIOInputs inputs) {}

    default void setDriveOpenLoop(double output) {}

    default void setTurnOpenLoop(double output) {}

    default void setDriveVelocity(double velocityRadPerSec) {}

    default void setTurnPosition(Rotation2d rotation) {}
}
