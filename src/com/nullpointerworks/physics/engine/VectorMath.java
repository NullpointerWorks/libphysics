package com.nullpointerworks.physics.engine;

import com.nullpointerworks.math.Approximate;

public class VectorMath 
{
	public static float[] create(float x, float y) 
	{
		return new float[] {x, y};
	}
	
	public static float[] copy(float[] v) 
	{
		return new float[] {v[0], v[1]};
	}
	
	public static float[] neg(float[] v) 
	{
		return new float[] {-v[0], -v[1]};
	}
	
	public static float[] add(float[] a, float[] b)
	{
		return new float[]{	a[0]+b[0],
							a[1]+b[1]};
	}
	
	public static float[] sub(float[] a, float[] b)
	{
		return new float[]{	a[0]-b[0],
							a[1]-b[1]};
	}
	
	public static float[] mul(float[] a, float f)
	{
		return new float[]{	a[0]*f,
							a[1]*f};
	}
	
	public static float cross(float[] a, float[] b) 
	{
		return a[0]*b[1]-a[1]*b[0];
	}
	
	public static float dot(float[] a, float[] b)
	{
		return a[0]*b[0] + a[1]*b[1];
	}
	
	public static float[] project(float[] A, float[] a, float lambda)
	{
		return add(A, mul(a, lambda) );
	}

	public static float magnitude(float[] v)
	{
		return (float) StrictMath.sqrt( dot(v,v) );
	}

	public static float[] normalize(float[] a)
	{
		float x = a[0];
		float y = a[1];
		float m = magnitude(a);
		float invm = 1f / m;
		return new float[]{	x*invm, y*invm};
	}

	public static float[] normal(float[] v, float f) 
	{
		return new float[] {-f*v[0], -f*v[1]};
	}
	
	public static float[] rotation(float angle)
	{
		float cs = (float)Approximate.cos(angle);
		float sn = (float)Approximate.sin(angle);
		return new float[]{cs,sn};
	}
}
