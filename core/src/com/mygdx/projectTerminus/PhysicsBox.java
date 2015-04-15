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
public class PhysicsBox {
    
    // Collision information
    public final double circleRadius;
    public Vector2[] vertices;
    
    // Whether the box is elastic
    public final boolean isElastic;
    
    // Transform properties (note that the position is in the centre of the box)
    public Vector2 position;
    public float rotation;
    public float mass;
    
    // Current velocity (linear and rotational) of the box
    public Vector2 velocity;
    public double omega;
    
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
                        Vector2 initVel, double initOmega, boolean elastic)
    {
        circleRadius = Math.sqrt(((sideLength / 2) * (sideLength / 2)) +
                        ((sideLength / 2) * (sideLength / 2)));
        isElastic = elastic;
        position = initPosition;
        rotation = initRotation;
        mass = initMass;
        
        velocity = initVel;
        omega = initOmega;
        
        //  The box's vertices are arranged as below.
        //
        //  V0 ________V1
        //    |        |
        //    |        |
        //    |        |
        //    |________|
        //  V3         V2
        vertices = new Vector2[4];
        vertices[0] = new Vector2(-sideLength / 2, sideLength / 2).add(position);
        vertices[1] = new Vector2(sideLength / 2, sideLength / 2).add(position);
        vertices[2] = new Vector2(sideLength / 2, -sideLength / 2).add(position);
        vertices[3] = new Vector2(-sideLength / 2, -sideLength / 2).add(position);

        updateVertices();
    }
    
    public void update(float deltaTime)
    {
        rotation += omega * deltaTime;
        position.x += velocity.x * deltaTime;
        position.y += velocity.y * deltaTime;
    }
    /**
     * Updates the box's vertex positions.
     */
    private void updateVertices()    
    {
        // Update all of the vertex positions based on the box's current position
        // and rotation
        for(Vector2 vertex : vertices)
        {
            vertex.sub(position);
            vertex.rotate(rotation);
            vertex.add(position);
        }
    }
    
}
