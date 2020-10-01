package org.firstinspires.ftc.teamcode.GOFUltimateGoal.Subsystems;

import java.util.Dictionary;
import java.util.Hashtable;

/**
 * A class to handle inter-subsystem communication.
 */
public class Handler {
    private static Handler handler = null;

    private Dictionary<Object, Object> data = new Hashtable<>();

    private Handler() {}

    /* Every subsystem HAS to have the same instance of the handler. */
    public static Handler getInstance() {
        if(handler == null) {
            handler = new Handler();
        }
        return handler;
    }

    public void pushData(Object caption, Object data) {
        this.data.put(caption, data);
    }

    public Object getData(Object key) {
        return data.get(key);
    }
}
