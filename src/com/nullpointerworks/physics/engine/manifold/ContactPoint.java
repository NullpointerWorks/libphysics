package com.nullpointerworks.physics.engine.manifold;

import static com.nullpointerworks.physics.engine.VectorMath.copy;

public class ContactPoint 
{
	private final float[] position;
	private final float[] normal;
	private final float penetration;
	
	public ContactPoint(float[] P, float[] N, float p)
	{
		position = P;
		normal = N;
		penetration = p;
	}
	
	public float[] getPosition()
	{
		return copy(position);
	}
	
	public float[] getNormal()
	{
		return copy(normal);
	}
	
	public float getPenetrationDepth()
	{
		return penetration;
	}
}
