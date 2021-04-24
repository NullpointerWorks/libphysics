package com.nullpointerworks.physics.engine;

import com.nullpointerworks.physics.engine.material.CustomMaterial;

public class MaterialFactory 
{
	public static CustomMaterial Custom(float density, float restitution, float sf, float kf)
	{return new CustomMaterial(density,restitution, sf,kf);}
	
	public static CustomMaterial Static()
	{return new CustomMaterial(0.0f, 0.05f, 0.6f, 0.4f);}
	
	public static CustomMaterial Light()
	{return new CustomMaterial(2.5f, 0.9f, 0.5f, 0.3f);}

	public static CustomMaterial Medium()
	{return new CustomMaterial(5f, 0.5f, 0.5f, 0.3f);}

	public static CustomMaterial Heavy()
	{return new CustomMaterial(10f, 0.1f, 0.5f, 0.3f);}
}
