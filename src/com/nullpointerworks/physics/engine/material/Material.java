package com.nullpointerworks.physics.engine.material;

public interface Material 
{
	float getDensity();
	float getRestitution();
	float getStaticFriction();
	float getDynamicFriction();
	Material getClone();
}
