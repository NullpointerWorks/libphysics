package com.nullpointerworks.physics.engine.material;

public interface Material 
{
	/**
	 * Returns the material density per unit volume.
	 * @return
	 */
	float getDensity();
	
	/**
	 * Returns the coefficient of restitution for this material.
	 * <pre>
	 * e = coefficient of restitution
	 * 
	 *     relative velocity after collision
	 * e = ----------------------------------
	 *     relative velocity before collision
	 * </pre>
	 * @return
	 */
	float getRestitution();
	
	/**
	 * Returns the 
	 * @return
	 */
	float getStaticFriction();
	
	/**
	 * 
	 * @return
	 */
	float getKineticFriction();
	
	/**
	 * 
	 * @return
	 */
	Material getClone();
}
