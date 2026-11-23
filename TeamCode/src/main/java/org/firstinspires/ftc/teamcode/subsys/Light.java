/** control GoBilda RGB indicator light using servo interface **/

package org.firstinspires.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hwScheduler.HWScheduler;
import org.firstinspires.ftc.teamcode.hwScheduler.SServo;

public class Light {
    // from: https://www.gobilda.com/rgb-indicator-light-pwm-controlled/
    double OFF = 0.0;
    double RED = 0.279;
    double ORANGE = 0.333;
    double YELLOW = 0.388;
    double GREEN = 0.5;
    double BLUE = 0.611;
    double VIOLET = 0.721;
    double WHITE = 1.0;

    private SServo light;

    public Light(HWScheduler hwScheduler) {
        light = new SServo("light", 0.5, hwScheduler.hw());
        hwScheduler.addDevice(light);

        off(); // start with light off
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