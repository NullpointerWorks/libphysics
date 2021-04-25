package com.nullpointerworks.physics.engine;

import java.util.HashMap;
import java.util.Map;

import com.nullpointerworks.physics.engine.material.Material;
import com.nullpointerworks.physics.engine.material.StaticMaterial;
import com.nullpointerworks.physics.engine.motion.AngularMotion;
import com.nullpointerworks.physics.engine.motion.LinearMotion;

import static com.nullpointerworks.physics.engine.math.MatrixMath.rotation;
import static com.nullpointerworks.physics.engine.math.VectorMath.create;
import static com.nullpointerworks.physics.engine.math.VectorMath.copy;
import static com.nullpointerworks.physics.engine.math.VectorMath.add;
import static com.nullpointerworks.physics.engine.math.VectorMath.cross;
import static com.nullpointerworks.physics.engine.math.VectorMath.project;

public class Composite
{
	/*
	 * composition
	 */
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
	public float[][] rotation;
	public float[] force;
	public float torque;
	public boolean immovable;
	public Map<Composite,Integer> ignore;
	
	public Composite()
	{
		material 	= null;
		shape 		= null;
		lmotion = new LinearMotion();
		amotion = new AngularMotion();
		
		
		force 		= create(0f, 0f); // gets reset per update
		torque 		= 0f; // gets reset per update
		rotation 	= rotation(0f);
		ignore 		= new HashMap<Composite,Integer>();
		immovable 	= false;
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
		
		c.rotation = rotation(amotion.getOrientation());
		c.force = copy(force);
		c.torque = torque;
		c.immovable = immovable;
		c.ignore = ignore;
		
		return c;
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
	public Composite setImmovable(boolean immovable) 
	{
		this.material = new StaticMaterial();
		this.immovable = immovable;
		compute();
		return this;
	}
	
	/**
	 * Add an entity to the blacklist to prevent interaction between the two.
	 */
	public Composite setBlacklist(Composite c)
	{
		if (ignore.containsKey(c)) return this;
		ignore.put(c, ignore.size());
		return this;
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
		float[] velocity = lmotion.getVelocity();
		float avelocity = amotion.getVelocity();
		
		velocity = project(velocity, impulse, inv_mass);
		avelocity = avelocity + ( inv_inertia * cross(contact, impulse) );
		
		lmotion.setVelocity(velocity);
		amotion.setVelocity(avelocity);
	}
	
	/**
	 * clear forces from this object
	 */
	public void clear() 
	{
		force 	= create(0f, 0f);
		torque 	= 0f;
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
		inv_mass = (m==0.0f)? 0.0f: 1f/m;
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
		inv_inertia = (Ip==0.0f)? 0.0f: 1f/Ip;
	}
}
