package org.firstinspires.ftc.teamcode.common;

@Deprecated
public class TelemetryVariable<E> {
    E[] value;

    public TelemetryVariable(E value) {
        this.value[0] = value;
    }
    public E getValue() {
        return value[0];
    }

    public void setValue(E val) {
        this.value[0] = val;
    }
}
