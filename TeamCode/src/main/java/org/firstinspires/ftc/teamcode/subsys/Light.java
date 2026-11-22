/** control GoBilda RGB indicator light using servo interface **/

package org.firstinspires.ftc.teamcode.subsys;

import org.firstinspires.ftc.teamcode.io.IoThread;
import org.firstinspires.ftc.teamcode.io.ServoDevice;
import org.firstinspires.ftc.teamcode.io.WritePolicy;

public class Light {
    /** configurables **/

    public static double PRIORITY_MULTIPLIER = 0.5;

    /** end configurables **/

    // from: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    double OFF = 0.0;
    double RED = 0.279;
    double ORANGE = 0.333;
    double YELLOW = 0.388;
    double GREEN = 0.5;
    double BLUE = 0.611;
    double VIOLET = 0.721;
    double WHITE = 1.0;

    private final ServoDevice light;

    public Light(IoThread io) {
        light = new ServoDevice(io, "light", OFF, new WritePolicy(PRIORITY_MULTIPLIER, 100)); // starts off
    }

    public void off() { light.setPosition(OFF); }

    /** color methods **/
    public void red() { light.setPosition(RED); }
    public void orange() { light.setPosition(ORANGE); }
    public void yellow() { light.setPosition(YELLOW); }
    public void green() { light.setPosition(GREEN); }
    public void blue() { light.setPosition(BLUE); }
    public void violet() { light.setPosition(VIOLET); }
    public void white() { light.setPosition(WHITE); }
}