package com.nullpointerworks.physics.engine.shape;

import com.nullpointerworks.math.FloatMath;

public class Circle extends Shape 
{
	private float area = 0f;
	public float radius = 0f;
	
	public Circle(float r)
	{
		radius = r;
		area = FloatMath.PI * radius * radius;
	}
	
	@Override
	public float area() 
	{
		return area;
	}
	
	@Override
	public float mass(float d) 
	{
		return area * d;
	}

	@Override
	public float inertia(float d) 
	{
		return mass(d) * radius * radius;
	}
	
	@Override
	public ShapeType type() 
	{
		return ShapeType.Circle;
	}

	@Override
	public Shape clone() 
	{
		return new Circle(radius);
	}
}
