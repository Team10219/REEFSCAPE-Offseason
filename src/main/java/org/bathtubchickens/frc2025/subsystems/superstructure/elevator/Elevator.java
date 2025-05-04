// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.subsystems.superstructure.elevator;

import org.bathtubchickens.frc2025.util.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Elevator {
    public static final double drumRadius = Units.inchesToMeters(0.877);
    private static final LoggedTunableNumber characterizationRampRate =
        new LoggedTunableNumber("Elevator/CharacterizationRampRate", 0.5);
    private static final LoggedTunableNumber characterizationUpVelocityThreshold = 
        new LoggedTunableNumber("Elevator/CharacterizationUpVelocityThreshold", 0.1);
    private static final LoggedTunableNumber characterizationDownStartAmps =
        new LoggedTunableNumber("Elevator/CharacterizationDownStartAmps", 0.0);
    private static final LoggedTunableNumber characterizationDownVelocityThreshold =
        new LoggedTunableNumber("Elevator/CharacterizationDownVelocityThreshold", -0.1);

    // don't use kI for elevators
    private static final LoggedTunableNumber kp =
        new LoggedTunableNumber("Elevator/kP");
    private static final LoggedTunableNumber kD =
        new LoggedTunableNumber("Elevator/kD");
    private static final LoggedTunableNumber kS = 
        new LoggedTunableNumber("Elevator/kS"); //0.5V
    private static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Elevator/kA"); //0.01 V*s^2/m
    private static final LoggedTunableNumber kG =
        new LoggedTunableNumber("Elevator/kG"); //0.11V
    private static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Elevator/kV"); //15.75V *s/m

}   
