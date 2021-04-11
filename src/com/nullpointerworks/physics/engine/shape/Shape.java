package com.nullpointerworks.physics.engine.shape;

public abstract class Shape 
{
	public Shape() { }
	public abstract ShapeType type();
	public abstract float area();
	public abstract float mass(float d);
	public abstract float inertia(float d);
	public abstract Shape clone();
}
