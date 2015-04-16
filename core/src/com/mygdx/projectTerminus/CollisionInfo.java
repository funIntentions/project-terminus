/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mygdx.projectTerminus;

import com.badlogic.gdx.math.Vector2;

/**
 * Contains the information about a collision: the point and time of collision,
 * objects after the objects have been moved back out of the collision, and the
 * 
 * @author Shane
 */
public class CollisionInfo {
    public final float time;
    public final Vector2 collPoints[];
    public final Vector2 p1;
    public final Vector2 p2;
    public final Vector2 normal;
    public final Vector2 tangential;
    
    public CollisionInfo(final float collisionTime, final Vector2 collisionPoints[],
            final Vector2 finalPosition1, final Vector2 finalPosition2,
            final Vector2 normalDirection, final Vector2 tangentialDirection)
    {
        time = collisionTime;
        collPoints = new Vector2[4];//new Vector2[collisionPoints.length];
        //System.arraycopy(collisionPoints, 0, collPoints, 0, collisionPoints.length);
        p1 = finalPosition1;
        p2 = finalPosition2;
        normal = normalDirection;
        tangential = tangentialDirection;
    }
}
