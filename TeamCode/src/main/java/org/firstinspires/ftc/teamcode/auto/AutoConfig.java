/** this class is used to provide auto parameters **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;

public class AutoConfig {
    public Pose pose1,
            pose2,
            pose3;

    public AutoConfig(
            Pose pose1,
            Pose pose2,
            Pose pose3
    ) {
        this.pose1 = pose1;
        this.pose2 = pose2;
        this.pose3 = pose3;
    }
}
