package com.nullpointerworks.physics.engine;

public interface Shape 
{
	ShapeType getType();
	float getArea();
	float getMass(float d);
	float getInertia(float d);
	Shape getClone();
}
