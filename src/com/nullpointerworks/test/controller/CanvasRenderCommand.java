package com.nullpointerworks.test.controller;

import com.nullpointerworks.color.ColorFormat;
import com.nullpointerworks.color.Colorizer;
import com.nullpointerworks.test.model.PhysicsLoop;
import com.nullpointerworks.test.view.GameView;

public class CanvasRenderCommand implements RenderCommand
{
	private GameView vWindow; 
	private PhysicsLoop mPhysiscSim;
	
	private Colorizer color;
	private int BLACK;
	private int WHITE;
	
	public CanvasRenderCommand(GameView v, PhysicsLoop s)
	{
		vWindow = v;
		mPhysiscSim = s;
		
		color = Colorizer.getColorizer( ColorFormat.RGB );
		BLACK = color.toInt(0,0,0);
		WHITE = color.toInt(255,255,255);
	}
	
	@Override
	public void onRender(double lerp) 
	{
		
		
		
		
		vWindow.setCanvasColor(BLACK);
		
	}
}
