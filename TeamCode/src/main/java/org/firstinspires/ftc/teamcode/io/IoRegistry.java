package org.firstinspires.ftc.teamcode.io;

/** Accepts devices that the I/O thread will own. Implemented by {@link IoThread}; tests use a plain list. */
interface IoRegistry {
    /**
     * @throws IllegalStateException if the I/O thread already started, or a device with the same name is registered
     */
    void register(IoDevice device);
}
