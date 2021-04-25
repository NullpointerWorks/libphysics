package com.nullpointerworks.physics.engine.material;

public class CustomMaterial implements Material
{
	private final float density;
	private final float restitution;
	private final float staticF;
	private final float kineticF;
	
	public CustomMaterial(float d, float r, float sF, float kF)
	{
		density 	= d;
		restitution = r;
		staticF 	= sF;
		kineticF 	= kF;
	}
	
	@Override
	public float getDensity() {return density;}
	
	@Override
	public float getRestitution() {return restitution;}
	
	@Override
	public float getStaticFriction() {return staticF;}
	
	@Override
	public float getKineticFriction() {return kineticF;}
	
	@Override
	public Material getClone() 
	{
		return new CustomMaterial(density, restitution, staticF, kineticF);
	}
}
