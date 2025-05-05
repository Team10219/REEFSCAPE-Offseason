// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.oi;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public interface DriverControls {
    public double getForward();

    public double getStrafe();

    public double getTurn();

    public Trigger resetFieldCentric();

    public Trigger autoAlign(BooleanSupplier left);
}
