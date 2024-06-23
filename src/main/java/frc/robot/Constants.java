// Copyright 2021-2024 FRC 6328
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

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Pivot {
    // Radians
    public static final double OUT = Units.degreesToRadians(180);
    public static final double IN = 0.0;
    public static final double THRESHOLD = 0.05;
    public static final double SIMOFFSET = Units.degreesToRadians(-125);
  }

  public static final class Spinner {
    public static final double ON = 0.8;
    public static final double FEEDING = -1;
    public static final double BACKWARDS = -0.3;
    public static final double THRESHOLD = 0.1;
  }

  public static final class Shooter {
    public static final double ON = 10;
    public static final double THRESHOLD = 0.1;
  }

  public static final class Amp {
    public static final double ON = 2;
    public static final double OUT = Units.degreesToRadians(120);
  }

  public static final class Climber {
    public static final double UP = 100;
  }
}
