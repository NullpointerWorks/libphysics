package com.nullpointerworks.physics.engine.shape;

import com.nullpointerworks.math.FloatMath;
import com.nullpointerworks.physics.engine.Shape;
import com.nullpointerworks.physics.engine.ShapeType;

public class Circle implements Shape 
{
	private float area = 0f;
	private float radius = 0f;
	
	public Circle(float r)
	{
		radius = r;
		area = FloatMath.PI * radius * radius;
	}
	
	public float radius()
	{
		return radius;
	}
	
	@Override
	public float getArea() 
	{
		return area;
	}
	
	@Override
	public float getMass(float d) 
	{
		return area * d;
	}

	@Override
	public float getInertia(float d) 
	{
		return getMass(d) * radius * radius;
	}
	
	@Override
	public ShapeType getType() 
	{
		return ShapeType.Circle;
	}

	@Override
	public Shape getClone() 
	{
		return new Circle(radius);
	}
}
