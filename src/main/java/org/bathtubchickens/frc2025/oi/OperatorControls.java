// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public interface OperatorControls {
    public Trigger intake();

    public Trigger outtake();

    public Trigger setLocationBottom();

    public Trigger setLocationLevel1();

    public Trigger setLocationLevel2();

    public Trigger setLocationLevel3();
}
