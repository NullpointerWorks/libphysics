package com.nullpointerworks.physics.engine.material;

public class HeavyMaterial implements Material
{
	private final float density;
	private final float restitution;
	private final float staticF;
	private final float kineticF;
	
	public HeavyMaterial()
	{
		density 	= 10f;
		restitution = 0.1f;
		staticF 	= 0.5f;
		kineticF 	= 0.3f;
	}
	
	@Override
	public float getDensity() {return density;}
	
	@Override
	public float getRestitution() {return restitution;}
	
	@Override
	public float getStaticFriction() {return staticF;}
	
	@Override
	public float getDynamicFriction() {return kineticF;}
	
	@Override
	public Material getClone() 
	{
		return new CustomMaterial(density, restitution, staticF, kineticF);
	}
}
