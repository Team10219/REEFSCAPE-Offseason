// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.bathtubchickens.frc2025.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class OperatorControlsXbox implements OperatorControls {
    private CommandXboxController operator;

    public OperatorControlsXbox(int port) {
        operator = new CommandXboxController(port);
    }

    @Override
    public Trigger intake() {
        return operator.a();
    }

    @Override
    public Trigger outtake() {
        return operator.rightTrigger();
    }

    @Override
    public Trigger setLocationBottom() {
        return operator.povUp();
    }

    @Override
    public Trigger setLocationLevel1() {
        return operator.povLeft();
    }

    @Override
    public Trigger setLocationLevel2() {
        return operator.povDown();
    }

    public Trigger setLocationLevel3() {
        return operator.povRight();
    }
}
