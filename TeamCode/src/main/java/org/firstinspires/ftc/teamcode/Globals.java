package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class Globals {

    public enum Location {
        LEFT, CENTER, RIGHT, // For preloads
        BLUE, RED, // Starting Alliance
        FAR, CLOSE, // Starting Side
    }

    public static Location SIDE = Location.FAR;
    /**
     * Match constants.
     */
    public static Location ALLIANCE = Location.RED;
}