/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.mygdx.projectTerminus;

import com.badlogic.gdx.math.Vector2;
import java.util.ArrayList;

/**
 *
 * @author Shane
 */
public class PhysicsBox extends RigidBody {
    
    // Collision information
    private final double circleRadius;
    
    // Whether the box is elastic
    public final boolean isElastic;
    
    // The length of one side of the box
    public float sideLen;
    
    // The positions of each vertex relative the centre
    private final Vector2[] initVertices;
    
    // The edges connecting the vertices
    private final ArrayList<Pair<Vector2, Vector2>> edges;
        
    /**
     * Creates a new PhysicsBox for collision with the awesome car.
     * @param initMass     The mass of the box.
     * @param sideLength   The length of each side of the box.
     * @param initPosition The box's initial position.
     * @param initRotation The box's initial rotation.
     * @param initVel      The box's initial linear velocity.
     * @param initOmega    The box's initial rotational velocity.
     * @param elastic      Whether the box is elastic.
     */
    public PhysicsBox(float sideLength,
                        Vector2 initPosition, float initRotation, float initMass,
                        Vector2 initVel, float initOmega, boolean elastic)
    {
        super(initMass, initPosition, initRotation, initVel, initOmega);
        
        circleRadius = Math.sqrt(((sideLength / 2) * (sideLength / 2)) +
                        ((sideLength / 2) * (sideLength / 2)));
        isElastic = elastic;
        sideLen = sideLength; 
        
        //  The box's vertices are arranged as below.
        //
        //  V0 ________V3
        //    |        |
        //    |        |
        //    |        |
        //    |________|
        //  V1         V2
        initVertices = new Vector2[4];
        initVertices[0] = new Vector2(-sideLength / 2, sideLength / 2);
        initVertices[1] = new Vector2(-sideLength / 2, -sideLength / 2);
        initVertices[2] = new Vector2(sideLength / 2, -sideLength / 2);
        initVertices[3] = new Vector2(sideLength / 2, sideLength / 2);
        
        vertices = new Vector2[4];
        edges = new ArrayList<Pair<Vector2, Vector2>>(4);
        
        for(int i = 0; i < initVertices.length; i++)
        {
            vertices[i] = new Vector2(initVertices[i]);
        }
        
        for(int i = 0; i < vertices.length; i++)
        {
            Vector2 nextV = i == vertices.length - 1 ? vertices[0] : vertices[i + 1];
            Pair<Vector2, Vector2> edge = new Pair<Vector2, Vector2>(vertices[i], nextV);
            edges.add(edge);
        }
        updateVertices();
    }
    
    @Override
    public void update(float deltaTime)
    {
        rotation += angularVelocity * deltaTime * (float)(180/Math.PI);
        position.x += velocity.x * deltaTime;
        position.y += velocity.y * deltaTime;

        System.out.println(deltaTime);
        System.out.println(velocity);
        System.out.println(position);

        updateVertices();
    }

    /**
     * Updates the box's vertex positions.
     */
    private void updateVertices()  
    {
        
        // Update all of the vertex positions based on the box's current position
        // and rotation
        for(int i = 0; i < vertices.length; i++)
        {
            vertices[i].x = initVertices[i].x;
            vertices[i].y = initVertices[i].y;
            vertices[i].rotate(rotation);
            vertices[i].x += position.x;
            vertices[i].y += position.y;
        }
    }
    
    @Override
    public Vector2[] getVertices()
    {
        return vertices;
    }
    
    @Override
    public ArrayList<Pair<Vector2, Vector2>> getEdges()
    {
        return edges;
    }
    
    @Override
    public double getBoundingCircleRadius()
    {
        return circleRadius;
    }
    
    @Override
    public Vector2 getCentreOfMass()
    {
        return position;
    }
    
    @Override
    public float getMomentOfInertia()
    {
        return (float)((this.mass * (Math.pow(sideLen, 2) + Math.pow(sideLen, 2)))
                / 12);
    }

    public void setAngularVelocity(float vel)
    {

    }
}
