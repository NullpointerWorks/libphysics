package com.nullpointerworks.physics.engine.material;

public class MediumMaterial implements Material
{
	private final float density;
	private final float restitution;
	private final float staticF;
	private final float kineticF;
	
	public MediumMaterial()
	{
		density 	= 5f;
		restitution = 0.6f;
		staticF 	= 0.6f;
		kineticF 	= 0.4f;
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
