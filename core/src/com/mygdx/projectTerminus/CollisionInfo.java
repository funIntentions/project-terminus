/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mygdx.projectTerminus;

import com.badlogic.gdx.math.Vector2;
import java.util.ArrayList;

/**
 * Contains the information about a collision: the point and time of collision,
 * objects after the objects have been moved back out of the collision, and the
 * 
 * @author Shane
 */
public class CollisionInfo {
    public final ArrayList<Vector2> manifold;
    public final float depth;
    public final Vector2 normal;
    public final Pair<RigidBody, RigidBody> bodies;
    
    public CollisionInfo(final Pair<RigidBody, RigidBody> collidingBodies,
                         final ArrayList<Vector2> collisionManifold,
                         final float PENETRATION_DEPTH,
                         final Vector2 normalDirection)
    {
        manifold = collisionManifold;
        normal = normalDirection;
        depth = PENETRATION_DEPTH;
        bodies = collidingBodies;
    }
    
    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("Manifold: ");
        sb.append(manifold);
        sb.append("\nNormal: ");
        sb.append(normal);
        sb.append("\nDepth: ");
        sb.append(depth);
        return sb.toString();
    }
}
