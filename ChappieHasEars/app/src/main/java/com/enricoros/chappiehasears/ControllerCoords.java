package com.enricoros.chappiehasears;

import java.io.Serializable;

/**
 * Current Mappings of the channels:
 *  0: left ear; -1: max back: 0: stop, 1: max forward
 *  1: right ear; -1: max back, 0: center, 1: max forward
 */
public class ControllerCoords implements Serializable {
    private static final long serialVersionUID = 1L;

    public static final float UNDEFINED = -2.0f;

    final float channel0;   // -1..0..1 (or UNDEFINED)
    final float channel1;

    ControllerCoords(float ch0, float ch1) {
        channel0 = ch0;
        channel1 = ch1;
    }
}
