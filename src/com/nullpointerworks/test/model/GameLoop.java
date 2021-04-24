package com.nullpointerworks.test.model;

import java.util.ArrayList;
import java.util.List;

import com.nullpointerworks.game.Asap;
import com.nullpointerworks.test.controller.RenderCommand;
import com.nullpointerworks.test.controller.UpdateCommand;

public class GameLoop extends Asap
{
	private List<UpdateCommand> updateObservers;
	private List<RenderCommand> renderObservers;
	
	public GameLoop()
	{
		updateObservers = new ArrayList<UpdateCommand>();
		renderObservers = new ArrayList<RenderCommand>();
	}
	
	public void addUpdateCommand(UpdateCommand uc)
	{
		updateObservers.add(uc);
	}
	
	public void addRenderCommand(RenderCommand rc)
	{
		renderObservers.add(rc);
	}
	
	@Override
	public void onInit() 
	{
		
	}
	
	@Override
	public void onUpdate(double dt)
	{
		for (UpdateCommand uc : updateObservers)
		{
			uc.onUpdate(dt);
		}
	}
	
	@Override
	public void onRender(double lerp) 
	{
		for (RenderCommand rc : renderObservers)
		{
			rc.onRender(lerp);
		}
	}
	
	@Override
	public void onDispose() 
	{
		
	}
}
