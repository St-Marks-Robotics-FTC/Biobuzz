package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * Localizer interface defining the core methods of the robot localization system.
 * All localizer implementations (e.g. wheel-encoder localization, vision localization) must implement this interface.
 *
 * Main responsibilities of a localizer:
 * 1. Track the robot field position and heading (pose)
 * 2. Provide the robot velocity estimate
 * 3. Allow manually setting the initial pose
 */
public interface Localizer {
    /**
     * Set the robot current pose.
     * Usually called at initialization or when localization must be reset.
     *
     *
     * @param pose pose to set, including x, y coordinates and heading angle
     */
    void setPose(Pose2d pose);

    /**
     * Get the current pose estimate.
     *
     * @return current pose estimate, including x, y coordinates and heading angle
     * @note This method does not update the pose estimate; call update() to update it
     */
    Pose2d getPose();

    /**
     * Update the localizer pose estimate.
     * Usually called once per loop, computing the new pose from sensor data.
     *
     * @return current velocity estimate, including linear and angular velocity
     */
    PoseVelocity2d update();
}
