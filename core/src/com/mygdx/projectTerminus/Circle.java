package com.mygdx.projectTerminus;

import com.badlogic.gdx.math.Vector2;

/**
 * Created by Dan on 3/9/2015.
 */
public class Circle
{
    public float mass;
    public Vector2 velocity;
    public Vector2 position;
    public float radius;

    public Circle(Vector2 pos, float m, Vector2 vel, float rad)
    {
        mass = m;
        velocity = new Vector2(vel);
        position = pos;
        radius = rad;
    }
}

