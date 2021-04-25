package com.nullpointerworks.physics.engine;

public interface Shape 
{
	ShapeType getType();
	float getArea();
	float getMass(float density);
	float getInertia(float density);
	Shape getClone();
}
