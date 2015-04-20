package com.mygdx.projectTerminus;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;
import com.sun.org.apache.xpath.internal.operations.Bool;
import java.util.ArrayList;

/**
 * Created by Dan on 4/5/2015.
 */
public class PhysicsRect extends RigidBody
{
    // Possible combinations of forces
    public static final int NO_FORCE = 0;
    public static final int TURNING_LEFT_FORCE = 1 << 0;
    public static final int TURNING_RIGHT_FORCE = 1 << 1;
    public static final int FORWARD_FORCE = 1 << 2;
    public static final int BACKWARD_FORCE = 1 << 3;
    
    // Force magnitudes
    public static final float LEFT_RIGHT_FORCE_MAG = 5000;
    public static final float BACK_FORWARD_FORCE_MAG = 10000;
    
    Vector2 COM;
    Vector2 radial;
    Vector2 forcePosition;
    Vector2 velocityTotal;
    float momentOfInertia;
    float width;
    float height;
    Color colour;
    Vector2 force;
    float linearAccel;
    float angularAccel;
    public float dragCoefficient = 250;
    private Array<PhysicsRect> childRects;
    private int currentForces;
    private double circleRadius;
    private final Vector2[] initialVerts;
    private final ArrayList<Pair<Vector2, Vector2>> edges;

    public PhysicsRect(float x, float y, float width, float height, Color colour, float mass, float rotation)
    {
        super(mass, new Vector2(x, y), rotation, new Vector2(0, 0), 0.f);
        
        velocityTotal =  new Vector2(0,0);
        this.width = width;
        this.height = height;
        this.colour = colour;
        childRects = new Array<PhysicsRect>();
        COM = new Vector2();
        force = new Vector2();
        radial = new Vector2();
        linearAccel = 0;
        momentOfInertia = 0;
        forcePosition = new Vector2();
                
        circleRadius = Math.sqrt(Math.pow(width / 2, 2) + Math.pow(height / 2, 2));
        
        initialVerts = new Vector2[4];
        initialVerts[0] = new Vector2(-width / 2, height / 2);
        initialVerts[1] = new Vector2(-width / 2, -height / 2);
        initialVerts[2] = new Vector2(width / 2, -height / 2);
        initialVerts[3] = new Vector2(width / 2, height / 2);
        
        vertices = new Vector2[4];
        edges = new ArrayList<Pair<Vector2, Vector2>>(initialVerts.length);
        for(int i = 0; i < vertices.length; i++)
        {
            vertices[i] = new Vector2(initialVerts[i]);
        }
        
        for(int i = 0; i < vertices.length; i++)
        {
            Vector2 nextV = i == vertices.length - 1 ? vertices[0] : vertices[i + 1];
            Pair<Vector2, Vector2> edge = new Pair<Vector2, Vector2>(vertices[i], nextV);
            edges.add(edge);
        }
        updateVertices();
    }

    public void reset(float x, float y, float width, float height, Color colour, float mass, float rotation)
    {
        position.x = x;
        position.y = y;
        velocity.x = 0.0f;
        velocity.y = 0.0f;
        angularVelocity = 0.0f;
        this.width = width;
        this.height = height;
        this.colour = colour;
        this.mass = mass;
        this.rotation = rotation;
        force = new Vector2();
        radial = new Vector2();
        linearAccel = 0;
        momentOfInertia = 0;
        forcePosition = Vector2.Zero ;
        velocityTotal = new Vector2(0,0);
    }

    public Vector2 backForwardForceLocation()
    {
        Vector2 pos = new Vector2(position.x - width/2, position.y);
        pos.sub(position);
        pos.rotate(rotation);
        pos.add(position);

        return pos;
    }

    public Vector2 turnLeftForceLocation()
    {
        Vector2 pos = new Vector2( position.x + width/2, position.y - height/2);
        pos.sub(position);
        pos.rotate(rotation);
        pos.add(position);
        return pos;
    }

    public Vector2 turnRightForceLocation()
    {
        Vector2 pos = new Vector2( position.x + width/2, position.y + height/2);
        pos.sub(position);
        pos.rotate(rotation);
        pos.add(position);
        return pos;
    }

    public void addChild(PhysicsRect rect)
    {
        childRects.add(rect);
    }

    public float getTotalMass()
    {
        float totalMass = this.mass;
        for (PhysicsRect child : childRects)
        {
            totalMass += child.mass;
        }

        return totalMass;
    }

    public void centralForceOn(Boolean forward)
    {
        currentForces |= forward ? FORWARD_FORCE: BACKWARD_FORCE;
    }

    public void centralForceOff(Boolean forward)
    {
        if (forward)
        {
            if ((currentForces & FORWARD_FORCE) != 0)
                currentForces ^= FORWARD_FORCE;
        }
        else
        {
            if ((currentForces & BACKWARD_FORCE) != 0)
                currentForces ^= BACKWARD_FORCE;
        }
    }

    public Boolean isForceOn(int force)
    {
        return ((currentForces & force) != 0);
    }

    public void rightForceOn()
    {
        currentForces |= TURNING_RIGHT_FORCE;
    }

    public void rightForceOff()
    {
        if ((currentForces & TURNING_RIGHT_FORCE) != 0)
            currentForces ^= TURNING_RIGHT_FORCE;
    }

    public Boolean isForceOn()
    {
        return currentForces != NO_FORCE;
    }

    public void leftForceOn()
    {
        currentForces |= TURNING_LEFT_FORCE;
    }

    public void leftForceOff()
    {
        if ((currentForces & TURNING_LEFT_FORCE) != 0)
            currentForces ^= TURNING_LEFT_FORCE;
    }

    @Override
    public float getMomentOfInertia()
    {
        float moi = 0;
        float Icm = (float)((mass * ((float)Math.pow(width, 2) + (float)Math.pow(height, 2))) / 12.0);
        float H = (float)(Math.sqrt(Math.pow(position.x - COM.x,2) + Math.pow(position.y - COM.y,2)));
        float I = Icm + (float)(mass * Math.pow(H, 2));

        moi += I;

        for (PhysicsRect rect : childRects)
        {
            Icm = (float)((rect.mass * ((float)Math.pow(rect.width, 2) + (float)Math.pow(rect.height, 2))) / 12.0);
            H = (float)(Math.sqrt(Math.pow(rect.position.x - COM.x,2) + Math.pow(rect.position.y - COM.y,2)));
            I = Icm + (float)(rect.mass * Math.pow(H, 2));

            moi += I;
        }
        return moi;
    }

    private float determineAngularAcceleration(float I)
    {
        Vector2 r = radial;
        Vector2 f = new Vector2(force);
        float t = r.crs(f);

        float angularAccel = 0.0f;
        angularAccel = t / I;

        return angularAccel;
    }

    @Override
    public double getBoundingCircleRadius()
    {
        return circleRadius;
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

    /**
     * Updates the physics simulation.
     * 
     * NOTE: I'm still not entirely sure the order in which things should be updated.
     * @param time The amount of time (in seconds) that has elapsed since the last
     * update.
     */
    @Override
    public void update(float time)
    {
        forcePosition.x = 0;
        forcePosition.y = 0;
        int numForces = 0;
        force.y = 0;
        force.x = 0;

        // Determine which forces are active and average their positions and
        // magnitudes
        if ((currentForces & TURNING_RIGHT_FORCE) != 0)
        {
            numForces++;

            forcePosition.x += position.x + width/2;
            forcePosition.y += position.y + height/2;
            force.x += LEFT_RIGHT_FORCE_MAG;
        }

        if ((currentForces & TURNING_LEFT_FORCE) != 0)
        {
            numForces++;

            forcePosition.x += position.x + width/2;
            forcePosition.y += position.y - height/2;
            force.x += LEFT_RIGHT_FORCE_MAG;
        }

        if ((currentForces & FORWARD_FORCE) != 0)
        {
            numForces++;

            forcePosition.x += position.x - width/2;
            forcePosition.y += position.y;
            force.x += BACK_FORWARD_FORCE_MAG;
        }

        if((currentForces & BACKWARD_FORCE) != 0)
        {
            numForces++;

            forcePosition.x += position.x - width/2;
            forcePosition.y += position.y;
            force.x -= BACK_FORWARD_FORCE_MAG;
        }

        // Average the forces
        numForces = numForces == 0 ? 1 : numForces; // Make this 1 to avoid dividing by 0
        forcePosition.x /= numForces;
        forcePosition.y /= numForces;

        forcePosition.sub(position);
        forcePosition.rotate(rotation);
        forcePosition.add(position);
        force.rotate(rotation);

        // Calculate COM
        float totalMass = mass;
        COM.x = position.x * mass;
        COM.y = position.y * mass;

        for (PhysicsRect rect : childRects)
        {
            COM.x += rect.position.x * rect.mass;
            COM.y += rect.position.y * rect.mass;
            totalMass += rect.mass;
        }
        if (totalMass > 0)
        {
            COM.x /= totalMass;
            COM.y /= totalMass;
        }

        // update vel, accel, and position
        radial.x = forcePosition.x - COM.x;
        radial.y = forcePosition.y - COM.y;

        Vector2 acceleration = new Vector2(force.x/totalMass, force.y/totalMass);
        this.momentOfInertia = getMomentOfInertia();

        float angularAcceleration = determineAngularAcceleration(momentOfInertia);

        // Determine Angular Velocity
        float angularForce = (angularAcceleration * totalMass);
        angularVelocity = 1/dragCoefficient * (angularForce - (float)Math.pow(Math.E, -dragCoefficient * time/totalMass) * (angularForce - dragCoefficient * angularVelocity));

        float rotationThisFrame =  angularVelocity  * time * (float)(180/Math.PI);
        rotation += rotationThisFrame;

        // Determine Velocity
        linearAccel = acceleration.x;
        velocity.x += acceleration.x * time;
        velocity.y += acceleration.y * time;

        // update position
        COM.x += velocity.x * time;
        COM.y += velocity.y * time;

        position.x += velocity.x * time;
        position.y += velocity.y * time;
        position.sub(COM);
        position.rotate(rotationThisFrame);
        position.add(COM);

        for (PhysicsRect rect : childRects)
        {
            rect.position.x += velocity.x * time;
            rect.position.y += velocity.y * time;
            rect.rotation = rotation;
            rect.position.sub(COM);
            rect.position.rotate(rotationThisFrame);
            rect.position.add(COM);
        }

        forcePosition.x  += velocity.x * time;
        forcePosition.y  += velocity.x * time;
        forcePosition.sub(COM);
        forcePosition.rotate(rotationThisFrame);
        forcePosition.add(COM);

        final float eToCoeff = (float)Math.pow(Math.E, -dragCoefficient * time/totalMass);
        velocity.x = 1/dragCoefficient * (force.x - eToCoeff * (force.x - dragCoefficient * velocity.x));
        velocity.y = 1/dragCoefficient * (force.y - eToCoeff * (force.y - dragCoefficient * velocity.y));
        
        updateVertices();
    }

    public void setAngularVelocity(float vel)
    {
//        float momentOfInertia = getMomentOfInertia();
//        this.momentOfInertia = momentOfInertia;
//
//        float angularAcceleration = determineAngularAcceleration(momentOfInertia);
//
//        // Determine Angular Velocity
//        float angularForce = (angularAcceleration * totalMass);
//
//        angularVelocity = 1/dragCoefficient * (angularForce - (float)Math.pow(Math.E, -dragCoefficient * time/totalMass) * (angularForce - dragCoefficient * angularVelocity));
    }
    
    /**
     * Updates the box's vertex positions.
     */
    private void updateVertices()    
    {
        for(int i = 0; i < vertices.length; i++)
        {
            vertices[i].x = initialVerts[i].x;
            vertices[i].y = initialVerts[i].y;
            vertices[i].rotate(rotation);
            vertices[i].x += position.x;
            vertices[i].y += position.y;
        }
    }
    
    @Override
    public Vector2 getCentreOfMass()
    {
        return COM;
    }
}
