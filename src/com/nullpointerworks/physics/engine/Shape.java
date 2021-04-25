package com.nullpointerworks.physics.engine;

public interface Shape 
{
	
	/**
	 * 
	 * @return
	 */
	ShapeType getType();
	
	/**
	 * 
	 * @return
	 */
	float getArea();
	
	/**
	 * 
	 * @return
	 */
	float getMass(float density);
	
	/**
	 * 
	 * @return
	 */
	float getInertia(float density);
	
	/**
	 * 
	 * @return
	 */
	Shape getClone();
}
