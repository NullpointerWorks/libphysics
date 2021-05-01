package com.nullpointerworks.physics.engine;

import static com.nullpointerworks.physics.engine.VectorMath.mul;
import static com.nullpointerworks.physics.engine.VectorMath.dot;

public class ImpulseMath 
{
	/**
	 * The allowable maximum difference between floating-point numbers to be considered the same. 
	 */
	public static final float EPSILON 		= 0.0001f;
	
	public static final float CORRECTION 	= 0.35f;
	public static final float ALLOWANCE 	= 0.05f;
	
	public static final float BIAS_RELATIVE = 0.95f;
	public static final float BIAS_ABSOLUTE = 0.01f;
	
	/**
	 * set the gravity resting constant for the engine
	 */
	public static float getRestingConstant(float[] g, float dt)
	{
		float[] gdt = mul(g, dt);
		return dot(gdt,gdt) + EPSILON;
	}
	
	/**
	 * true if the given values are close enough together.
	 */
	public static boolean isEqual(float a, float b)
	{
		return StrictMath.abs(a - b) <= EPSILON;
	}
	
	/**
	 * true if the given values are close enough together.
	 */
	public static boolean isZero(float a)
	{
		return a <= EPSILON;
	}
	
	/**
	 * gt
	 */
	public static boolean bias(float a, float b)
	{
		return (b * BIAS_RELATIVE + a * BIAS_ABSOLUTE) < a;
	}
}
