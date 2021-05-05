package com.nullpointerworks.test.model;

import com.nullpointerworks.game.Loop;

public abstract class ManagedLoop implements Runnable, Loop 
{
	private Thread thread;
	private boolean running = true;
	private int target_update = 30; 
	private long ideal_time = NANO / target_update;
	private long nanotime_prev;
	
	@Override
	public void setTargetFPS(int fps) 
	{
		target_update = fps;
		ideal_time = NANO / target_update;
	}
	
	@Override
	public void setTargetHz(double hertz) 
	{
		setTargetFPS( (int)(hertz+0.5) );
	}
	
	@Override
	public void start()
	{
		thread = new Thread(this);
		thread.start();
	}
	
	@Override
	public void stop()
	{
		running = false;
	}
	
	@Override
	public void run()
	{
		onInit();
		
		long nanotime_curr;
		long nanotime_delta;
		long sleep;
		double timing;
		ideal_time = NANO / target_update;
		nanotime_prev = System.nanoTime();
		
		while (running)
		{
			nanotime_curr = System.nanoTime();
			nanotime_delta = nanotime_curr - nanotime_prev;
			nanotime_prev = nanotime_curr;
			timing = nanotime_delta * inv_NANO;
			onUpdate(timing);
			
			/*
			 * get nanotime till next cycle
			 * convert to milliseconds, since thats what sleep() wants
			 */
			sleep = (long)( ((nanotime_curr - System.nanoTime() + ideal_time) * inv_MICRO));
			try
			{
				Thread.sleep( (sleep<0)?0:sleep );
			} 
			catch (InterruptedException e) 
			{
				e.printStackTrace();
			}
		}
		
		onDispose();
	}
	
	@Override
	public final void onRender(double lerp) { }
}
