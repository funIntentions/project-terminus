package com.mygdx.projectTerminus;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;

/**
 * Created by Dan on 4/5/2015.
 */
public class PhysicsRect
{
    Vector2 position;
    Vector2 COM;
    Vector2 radial;
    Vector2 forcePosition;
    Vector2 velocity;
    Vector2 velocityTotal;
    float momentOfInertia;
    float angularVelocity;
    float mass;
    float width;
    float height;
    float rotation;
    Color colour;
    Vector2 force;
    float linearAccel;
    float angularAccel;
    public float dragCoefficient = 200;
    private Array<PhysicsRect> childRects;
    private Boolean forceActing;

    public PhysicsRect(float x, float y, float width, float height, Color colour, float mass, float rotation)
    {
        position = new Vector2(x,y);
        velocity = new Vector2(0,0);
        velocityTotal =  new Vector2(0,0);
        angularVelocity = 0.0f;
        this.width = width;
        this.height = height;
        this.colour = colour;
        this.mass = mass;
        this.rotation = rotation;
        childRects = new Array<PhysicsRect>();
        COM = new Vector2();
        force = new Vector2();
        radial = new Vector2();
        linearAccel = 0;
        momentOfInertia = 0;
        forcePosition = new Vector2();
        forceActing = false;
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
        forceActing = true;
        forcePosition.x = position.x - width/2;
        forcePosition.y = position.y;
        forcePosition.sub(position);
        forcePosition.rotate(rotation);
        forcePosition.add(position);
        force.x = forward ? 10000 : -10000;
        force.y = 0;
        force.rotate(rotation);
    }

    public void rightForceOn()
    {
        forceActing = true;
        forcePosition.x = position.x + width/2;
        forcePosition.y = position.y + height/2;
        forcePosition.sub(position);
        forcePosition.rotate(rotation);
        forcePosition.add(position);
        force.x = 5000;
        force.y = 0;
        force.rotate(rotation);
    }

    public void forceOff() {
        forceActing = false;
        force.x = 0;
        force.y = 0;
    }

    public Boolean isForceOn()
    {
        return forceActing;
    }

    public void leftForceOn()
    {
        forceActing = true;
        forcePosition.x = position.x + width/2;
        forcePosition.y = position.y - height/2;
        forcePosition.sub(position);
        forcePosition.rotate(rotation);
        forcePosition.add(position);
        force.x = 5000;
        force.y = 0;
        force.rotate(rotation);
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

    public void update(float time)
    {
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

        float momentOfInertia = determineMomentOfIntertia();
        this.momentOfInertia = momentOfInertia;

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

        velocity.x = 1/dragCoefficient * (force.x - (float)Math.pow(Math.E, -dragCoefficient * time/totalMass) * (force.x - dragCoefficient * velocity.x));
        velocity.y = 1/dragCoefficient * (force.y - (float)Math.pow(Math.E, -dragCoefficient * time/totalMass) * (force.y - dragCoefficient * velocity.y));
    }
}

