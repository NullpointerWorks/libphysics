package com.nullpointerworks.test.controller;

import java.util.List;

import com.nullpointerworks.color.ColorFormat;
import com.nullpointerworks.color.Colorizer;
import com.nullpointerworks.core.buffer.IntBuffer;
import com.nullpointerworks.physics.engine.Composite;
import com.nullpointerworks.test.model.PhysicsLoop;
import com.nullpointerworks.test.view.GameView;

public class CanvasRenderCommand implements RenderCommand
{
	private GameView vWindow; 
	private PhysicsLoop mPhysiscSim;
	
	private IntBuffer canvas;
	private Colorizer color;
	private int BLACK;
	private int WHITE;
	
	public CanvasRenderCommand(GameView v, PhysicsLoop s)
	{
		vWindow = v;
		mPhysiscSim = s;
		
		canvas = v.getCanvas();
		color = Colorizer.getColorizer( ColorFormat.RGB );
		BLACK = color.toInt(0,0,0);
		WHITE = color.toInt(255,255,255);
	}
	
	@Override
	public void onRender(double lerp) 
	{
		List<Composite> bodies = mPhysiscSim.getBodies();
		canvas.clear(BLACK);
		
		
		for (Composite c : bodies)
		{
			c.getShape().getType();
		}
		
		
		vWindow.swapCanvas();
	}
}
