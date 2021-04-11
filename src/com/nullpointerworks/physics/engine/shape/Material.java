package com.nullpointerworks.physics.engine.shape;

public class Material 
{
	public final float density;
	public final float restitution;
	public final float staticF;
	public final float kineticF; // dynamic/sliding friction
	
	private Material(float d, float r)
	{
		this(d,r, 0.5f,0.3f);
	}
	
	private Material(float d, float r, float sF, float kF)
	{
		density 	= d;
		restitution = r;
		staticF 	= sF;
		kineticF 	= kF;
	}
	
	// ================== custom =======================

	public static Material New(float density, float restitution)
	{return new Material(density,restitution);}
	
	public static Material New(float density, float restitution, float sf, float kf)
	{return new Material(density,restitution, sf,kf);}
	
	public static Material Static()
	{return new Material(0.0f, 0.05f, 0.6f, 0.4f);}
	
	public static Material Light()
	{return new Material(2.5f, 0.9f, 0.5f, 0.3f);}

	public static Material Medium()
	{return new Material(5f, 0.5f, 0.5f, 0.3f);}

	public static Material Heavy()
	{return new Material(10f, 0.1f, 0.5f, 0.3f);}
}