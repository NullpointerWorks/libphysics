package com.nullpointerworks.physics.engine;

import static com.nullpointerworks.physics.engine.VectorMath.add;
import static com.nullpointerworks.physics.engine.VectorMath.copy;
import static com.nullpointerworks.physics.engine.VectorMath.create;
import static com.nullpointerworks.physics.engine.VectorMath.cross;
import static com.nullpointerworks.physics.engine.VectorMath.project;

import java.util.HashMap;
import java.util.Map;

import com.nullpointerworks.physics.engine.material.Material;
import com.nullpointerworks.physics.engine.motion.AngularMotion;
import com.nullpointerworks.physics.engine.motion.LinearMotion;

public class Composite
{
	private Material material;
	private Shape shape;
	private LinearMotion lmotion;
	private AngularMotion amotion;
	
	/*
	 * mass
	 */
	public float mass;
	public float inertia;
	public float inv_mass;
	public float inv_inertia;
	
	/*
	 * other data
	 */
	public float[] force;
	public float torque;
	private boolean immovable;
	private Map<Composite,Integer> ignore;
	
	public Composite()
	{
		material 	= null;
		shape 		= null;
		lmotion 	= new LinearMotion();
		amotion 	= new AngularMotion();
		
		ignore 		= new HashMap<Composite,Integer>();
		immovable 	= false;
		
		force 		= create(0f, 0f); // gets reset per update
		torque 		= 0f; // gets reset per update
	}
	
	/**
	 * 
	 * @return
	 */
	public Shape getShape() {return shape;}
	
	/**
	 * 
	 * @return
	 */
	public Material getMaterial() {return material;}
	
	/**
	 * 
	 * @return
	 */
	public LinearMotion getLinearMotion() {return lmotion;}
	
	/**
	 * 
	 * @return
	 */
	public AngularMotion getAngularMotion() {return amotion;}
	
	/**
	 * 
	 * @return a copy of this entity
	 */
	public Composite getCopy()
	{
		Composite c = new Composite();
		c.setShape( shape.getClone() );
		c.setMaterial( material.getClone() );
		c.setLinearMotion( lmotion.getClone() );
		c.setAngularMotion( amotion.getClone() );
		
		c.immovable = immovable;
		c.ignore = ignore;
		
		c.force = copy(force);
		c.torque = torque;
		
		return c;
	}
	
	/**
	 * 
	 * @return
	 */
	public boolean isImmovable() 
	{
		return immovable;
	}
	
	// =================================================
	
	/**
	 * set a material for this entity
	 */
	public void setMaterial(Material mat)
	{
		if (immovable) return;
		material = mat.getClone();
		compute();
	}
	
	/**
	 * provide a shape for the entity
	 */
	public void setShape(Shape sh)
	{
		shape = sh.getClone();
		compute();
	}
	
	/**
	 * 
	 * @return
	 */
	public void setLinearMotion(LinearMotion lm)
	{
		lmotion = lm.getClone();
	}
	
	/**
	 * 
	 * @return
	 */
	public void setAngularMotion(AngularMotion am)
	{
		amotion = am.getClone();
	}
	
	/**
	 * set this entity as immovable by the engine
	 */
	public void setImmovable(boolean immovable) 
	{
		this.immovable = immovable;
	}
	
	/**
	 * Add an entity to the blacklist to prevent interaction between the two.
	 */
	public void setBlacklist(Composite c)
	{
		if (ignore.containsKey(c)) return;
		ignore.put(c, ignore.size());
	}
	
	// =================================================
	
	/**
	 * check to see if the Composite has been blacklisted for collision checks
	 */
	public boolean contains(Composite c)
	{
		return ignore.containsKey(c);
	}
	
	/**
	 * add the given force vector to the body
	 */
	public void applyForce(float[] f)
	{
		force = add(force, f);
	}
	
	/**
	 * <pre>
	 * velocity = impulse / mass<br>
	 * angular velocity = (contact x impulse) / inertia
	 * </pre>
	 */
	public void applyImpulse(float[] impulse, float[] contact)
	{
		float[] lvelocity = lmotion.getVelocity();
		float avelocity = amotion.getVelocity();
		
		lvelocity = project(lvelocity, impulse, inv_mass);
		avelocity = avelocity + ( cross(contact, impulse) * inv_inertia );
		
		lmotion.setVelocity(lvelocity);
		amotion.setVelocity(avelocity);
	}
	
	/**
	 * clear forces from this object
	 */
	public void clear() 
	{
		force = create(0f, 0f);
		torque = 0f;
	}
	
	// ====================================================
	
	/*
	 * compute mass and moment of inertia
	 */
	private void compute() 
	{
		if (shape == null) return;
		if (material == null) return;
		
		float density = material.getDensity();
		setMass(shape.getMass(density) );
		setInertia(shape.getInertia(density) );
	}
	
	/**
	 * calculate the mass<br>
	 * d = density<br>
	 * V = volume<br>
	 * <pre>
	 * mass = density * volume
	 * </pre>
	 */
	private void setMass(float m)
	{
		mass = m;
		inv_mass = (m==0.0f)? Float.MAX_VALUE: 1f/m;
	}
	
	/**
	 * set the moment of inertia. This value comes from the Shape class.
	 * <pre>
	 * Ip = SUM(m*r^2)
	 * </pre>
	 */
	private void setInertia(float Ip)
	{
		inertia = Ip;
		inv_inertia = (Ip==0.0f)? Float.MAX_VALUE: 1f/Ip;
	}
}
