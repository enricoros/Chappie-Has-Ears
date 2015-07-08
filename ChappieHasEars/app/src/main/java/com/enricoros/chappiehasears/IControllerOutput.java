package com.enricoros.chappiehasears;

public interface IControllerOutput {

	/**
	 * Controls the motion of the platform
	 *
	 * @param coordinates The coordinates of the motion event.
	 */
	public void setCoordinates(ControllerCoords coordinates);

}
