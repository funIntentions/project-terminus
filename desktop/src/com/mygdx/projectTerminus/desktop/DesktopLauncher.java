package com.mygdx.projectTerminus.desktop;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;
import com.mygdx.projectTerminus.Game;

public class DesktopLauncher {
	public static void main (String[] arg) {
		LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
        config.title = "Terminus";
        config.height = 800;
        config.width = 800;
        new LwjglApplication(new Game(), config);
	}
}
