package com.nullpointerworks.physics.engine;

import static com.nullpointerworks.physics.engine.VectorMath.mul;
import static com.nullpointerworks.physics.engine.VectorMath.dot;

public class ImpulseMath 
{
	public static final float EPSILON 		= 0.0001f;
	
	public static final float CORRECTION 	= 0.5f;
	public static final float ALLOWANCE 	= 0.04f;
	
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
	public static boolean equal(float a, float b)
	{
		return StrictMath.abs(a - b) <= EPSILON;
	}
	
	/**
	 * 
	 */
	public static boolean gt(float a, float b)
	{
		return (b * BIAS_RELATIVE + a * BIAS_ABSOLUTE) < a;
	}
}