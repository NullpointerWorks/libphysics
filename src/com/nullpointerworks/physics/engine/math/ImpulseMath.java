package com.nullpointerworks.physics.engine.math;

import com.nullpointerworks.math.vector.Vector2;

public class ImpulseMath 
{
	private static final Vector2 vec2 = Vector2.New();
	
	public static final float EPSILON 		= 0.0001f;
	
	public static final float CORRECTION 	= 0.5f;
	public static final float ALLOWANCE 	= 0.04f;
	
	public static final float BIAS_RELATIVE = 0.95f;
	public static final float BIAS_ABSOLUTE = 0.01f;
	
	public static float RESTING = EPSILON;
	
	/**
	 * set the gravity resting constant for the engine
	 */
	public static void applyRestingConstant(float[] g, float dt)
	{
		float[] gdt = vec2.mul(g, dt);
		RESTING = vec2.dot(gdt,gdt) + EPSILON;
		gdt = null;
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
