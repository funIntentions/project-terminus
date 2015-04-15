/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mygdx.projectTerminus;

import com.badlogic.gdx.math.Vector2;

/**
 *
 * @author Shane
 */
public abstract class RigidBody {
    public float mass;
    public Vector2 position;
    public float rotation;
    public Vector2 velocity;
    public float angularVelocity;
    public Vector2 vertices[];
    
    public RigidBody(float initMass, Vector2 initPosition, float initRotation,
                       Vector2 initVel, float initAngleVel)
    {
        mass = initMass;
        position = initPosition;
        rotation = initRotation;
        velocity = initVel;
        angularVelocity = initAngleVel;
    }
    
    /**
     * Gets the list of vertices for the given rigid object.
     * @return The list of vertices for this rigid object.
     */
    public abstract Vector2[] getVertices();
    
    /**
     * Gets the radius of the bounding circle to be used for broadphase collision
     * detection.
     * @return The radius of the object's bounding circle.
     */
    public abstract double getBoundingCircleRadius();
    
    /**
     * Update this object's physical state with the given time interval.
     * @param time The time interval over which to integrate.
     */
    public abstract void update(float time);
}
