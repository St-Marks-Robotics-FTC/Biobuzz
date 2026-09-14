/** base red auto **/

package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.math.Pose;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public abstract class RedAuto extends BozoAuto { // these positions override the base auto class
    public static Pose startPose = new Pose (0, 0, 0);
    public static Pose shootLeftPose = new Pose (0, 0, 0);
    public static Pose shootRightPose = new Pose (0, 0, 0);
    public static Pose flowerLeftPose = new Pose (0, 0, 0);
    public static Pose flowerRightPose = new Pose (0, 0, 0);

    @Override
    protected AutoConfig buildConfig() {
        return new AutoConfig(
                startPose,
                shootLeftPose,
                shootRightPose,
                flowerLeftPose,
                flowerRightPose
        );
    }
}
