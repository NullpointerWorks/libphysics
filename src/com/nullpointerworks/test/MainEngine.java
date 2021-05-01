package com.nullpointerworks.test;

import com.nullpointerworks.physics.PlanetaryGravitation;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.physics.engine.Shape;
import com.nullpointerworks.physics.engine.PolygonBuilder;
import com.nullpointerworks.physics.engine.VectorMath;
import com.nullpointerworks.physics.engine.material.*;
import com.nullpointerworks.physics.engine.shape.*;

import com.nullpointerworks.test.controller.CanvasRenderCommand;
import com.nullpointerworks.test.controller.RenderCommand;
import com.nullpointerworks.test.model.GameLoop;
import com.nullpointerworks.test.model.PhysicsLoop;
import com.nullpointerworks.test.view.GameView;

public class MainEngine
{
	public static void main(String[] args) 
	{
		new MainEngine();
	}
	
	public MainEngine()
	{
		PhysicsLoop mPhysiscSim 		= new PhysicsLoop();
		GameLoop mGameSim 				= new GameLoop();
		GameView vWindow 				= new GameView(800, 600);
		RenderCommand cRenderToScreen 	= new CanvasRenderCommand(vWindow, mPhysiscSim);
		
		mPhysiscSim.setTargetFPS(100);
		mPhysiscSim.setGravity( VectorMath.create(0f, -10f) , (float)PlanetaryGravitation.EARTH );
		mPhysiscSim.setIntegrationCycles(20);
		
		mGameSim.setTargetFPS(30);
		mGameSim.addRenderCommand(cRenderToScreen);
		
		mPhysiscSim.start();
		mGameSim.start();
		vWindow.setVisible(true);
		
		
		
		makeBox(mPhysiscSim, 280f, 350f, 0f, new HeavyMaterial(), false);
		//makeBox(mPhysiscSim, 50f, 0f, new StaticMaterial(), true);
		
		
		//makeCirle(mPhysiscSim, 20f, 300f, 300f, new LightMaterial(), false);
		makeCirle(mPhysiscSim, 100f, 240f, 100f, new StaticMaterial(), true);
		makeCirle(mPhysiscSim, 100f, 460f, 100f, new StaticMaterial(), true);
		
	}
	
	private void makeCirle(PhysicsLoop sim, float size, float x, float y, Material mat, boolean immovable) 
	{
		Composite circle = new Composite();
		circle.setShape( new Circle( size ) );
		circle.setMaterial( mat );
		circle.setImmovable(immovable);
		
		circle.getLinearMotion().setPosition( new float[] {x, y} );
		
		sim.addComposite(circle);
	}
	
	private void makeBox(PhysicsLoop sim, float x, float y, float r, Material mat, boolean immovable) 
	{
		Shape polygon = PolygonBuilder.Box(200f, 40f);
		
		Composite box = new Composite();
		box.setShape( polygon );
		box.setMaterial( mat );
		box.setImmovable(immovable);
		
		box.getLinearMotion().setPosition( new float[] {x,y} );
		box.getAngularMotion().setOrientation( r * 0.017452778f ); // 0.01745277777777777777777777777778 = PI / 180 deg
		
		sim.addComposite(box);
	}
}
