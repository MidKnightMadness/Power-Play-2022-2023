package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class TelemetrySystem {
    ArrayList<String> staticMessages;
    boolean staticMessagesHaveChanged = false;

    ArrayList dynamicMessages;

    Telemetry telemetry;

    public TelemetrySystem(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int addDynamicMessage(String staticCaption, TelemetryVariable variable) {
        dynamicMessages.add(staticCaption);
        dynamicMessages.add(variable);

        return dynamicMessages.size() - 1;

    }

    public void updateDynamicMessages() {

    }

    public int addStaticMessage(String message) {
        staticMessages.add(message);
        return staticMessages.size() - 1;
    }

    public void setStaticMessage(int i, String message) {
        staticMessages.set(i, message);
    }

    public void setStaticMessages(ArrayList<String> messages) {
        this.staticMessages = messages;
    }

    public boolean removeMessage(String message) {
        return staticMessages.remove(message);
    }

    public void removeMessageAt(int i) {
        staticMessages.remove(i);
    }

    public ArrayList<String> getStaticMessages() {
        return staticMessages;
    }

    public String stringRepr() {
        String result = "";

        for (String message : staticMessages) {
            result += message + "/n";
        }

        return result;
    }

    public void update() {
        if (!staticMessagesHaveChanged) {
            return;
        }

        staticMessagesHaveChanged = false;
    }

}
