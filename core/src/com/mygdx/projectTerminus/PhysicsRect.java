package com.mygdx.projectTerminus;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;

/**
 * Created by Dan on 4/5/2015.
 */
public class PhysicsRect extends RigidObject
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
    public float dragCoefficient = 200;
    private Array<PhysicsRect> childRects;
    private int currentForces;

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

    public void addChild(PhysicsRect rect)
    {
        childRects.add(rect);
    }


    public void centralForceOn(Boolean forward)
    {
        currentForces |= forward ? FORWARD_FORCE: BACKWARD_FORCE;
//        forcePosition.x = position.x - width/2;
//        forcePosition.y = position.y;
//        forcePosition.sub(position);
//        forcePosition.rotate(rotation);
//        force.x = forward ? 10000 : -10000;
//        force.y = 0;
//        force.rotate(rotation);
    }

    public void rightForceOn()
    {
        currentForces |= TURNING_RIGHT_FORCE;
//        forceActing = true;
//        forcePosition.x = position.x + width/2;
//        forcePosition.y = position.y + height/2;
//        forcePosition.sub(position);
//        forcePosition.rotate(rotation);
//        forcePosition.add(position);
//        force.x = 5000;
//        force.y = 0;
//        force.rotate(rotation);
    }

    public void forceOff() {
        currentForces &= 0;
    }

    public Boolean isForceOn()
    {
        return currentForces != NO_FORCE;
    }

    public void leftForceOn()
    {
        currentForces |= TURNING_LEFT_FORCE;
//        forceActing = true;
//        forcePosition.x = position.x + width/2;
//        forcePosition.y = position.y - height/2;
//        forcePosition.sub(position);
//        forcePosition.rotate(rotation);
//        forcePosition.add(position);
//        force.x = 5000;
//        force.y = 0;
//        force.rotate(rotation);
    }

    private float determineMomentOfIntertia()
    {
        float momentOfInertia = 0;
        float Icm = (float)((mass * ((float)Math.pow(width, 2) + (float)Math.pow(height, 2))) / 12.0);
        float H = (float)(Math.sqrt(Math.pow(position.x - COM.x,2) + Math.pow(position.y - COM.y,2)));
        float I = Icm + (float)(mass * Math.pow(H, 2));

        momentOfInertia += I;

        for (PhysicsRect rect : childRects)
        {
            Icm = (float)((rect.mass * ((float)Math.pow(rect.width, 2) + (float)Math.pow(rect.height, 2))) / 12.0);
            H = (float)(Math.sqrt(Math.pow(rect.position.x - COM.x,2) + Math.pow(rect.position.y - COM.y,2)));
            I = Icm + (float)(rect.mass * Math.pow(H, 2));

            momentOfInertia += I;
        }


        return momentOfInertia;
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
        return 0.0;
    }
    
    @Override
    public Vector2[] getVertices()
    {
        return null;
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
        Vector2 force = new Vector2(1, 0);   // The unit vector in the x direction will be rotated
        Vector2 forcePos = new Vector2(0,0); // Force is applied here
        float forceMagnitude = 0;
        int numForces = 0;
        
        // Determine which forces are active and average their positions and
        // magnitudes
        if ((currentForces & TURNING_RIGHT_FORCE) != 0)
        {
            forcePos.add(new Vector2(width / 2, height / 2));
            forceMagnitude += LEFT_RIGHT_FORCE_MAG;
            numForces++;
        }
        
        if ((currentForces & TURNING_LEFT_FORCE) != 0)
        {
            forcePos.add(new Vector2(width / 2, -height / 2));
            forceMagnitude += LEFT_RIGHT_FORCE_MAG;
            numForces++;
        }
        
        if ((currentForces & FORWARD_FORCE) != 0)
        {
            forcePos.add(new Vector2(-width / 2, 0));
            forceMagnitude += BACK_FORWARD_FORCE_MAG;
            numForces++;
        }
        
        if((currentForces & BACKWARD_FORCE) != 0)
        {
            forcePos.add(new Vector2(-width / 2, 0));
            forceMagnitude -= BACK_FORWARD_FORCE_MAG;
            numForces++;
        }
        
        numForces = numForces == 0 ? 1 : numForces; // Make this 1 to avoid dividing by 0
        // Average forces
        forceMagnitude /= numForces;
        forcePos.x /= numForces;
        forcePos.y /= numForces;
        forcePos.rotate(rotation);
        forcePos.add(position);
        forcePosition = forcePos;
        
        // Point force in the correct direction and give it the right magnitude
        force.rotate(rotation);
        force.x *= forceMagnitude;
        force.y *= forceMagnitude;
        
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
        radial.x = forcePos.x - COM.x;
        radial.y = forcePos.y - COM.y;
        
        // Calculate displacement
        Vector2 displacement = new Vector2(velocity.x * time, velocity.y * time);
        
        // Calculate new linear velocity
        final Vector2 constVo = new Vector2(velocity.x * dragCoefficient, velocity.y * dragCoefficient); // Cvo
        final float eExponent = (-dragCoefficient * time) / totalMass; // -Ct/m
        final Vector2 tMinusCvo = new Vector2(force).sub(constVo); // T - Cvo
        final Vector2 secondTerm = new Vector2(tMinusCvo.x * (float)Math.pow(Math.E, eExponent),
                                               tMinusCvo.y * (float)Math.pow(Math.E, eExponent)); // e^(-Ct/m)(T - Cvo)
        final Vector2 newVel = new Vector2(force).sub(secondTerm);
        newVel.x *= 1 / dragCoefficient;
        newVel.y *= 1 / dragCoefficient;
        velocity = newVel;
        
        // Update linear position
        COM.add(displacement);
        position.add(displacement);

//        Vector2 acceleration = new Vector2(force.x/totalMass, force.y/totalMass);
      
//        // Determine the new angular velocity
//        float curMomentOfInertia = determineMomentOfIntertia();
//        float angularAcceleration =  new Vector2(radial).crs(force) / curMomentOfInertia;
//        
//        // Set the moment of inertia, angular accel and velocity
//        this.momentOfInertia = curMomentOfInertia;
//        this.angularAccel = angularAcceleration;
//        this.angularVelocity += angularAcceleration * time;
//        
//        // Determine the change in angle 
//        float deltaTheta = this.angularVelocity * time;
//        this.rotation += deltaTheta;
//        
//        // Rotate the position about the centre of mass
//        position.sub(COM);
//        position.rotate(deltaTheta);
//        position.add(COM);
//
//        // Update the child rectangles (the gas tank and passenger/driver)
//        for (PhysicsRect rect : childRects)
//        {
//            rect.position.add(displacement);
//            rect.rotation = this.rotation;
//            rect.position.sub(COM);
//            rect.position.rotate(deltaTheta);
//            rect.position.add(COM);
//        }
    }
}
