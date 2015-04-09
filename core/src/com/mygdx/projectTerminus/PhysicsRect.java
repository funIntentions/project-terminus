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
    float momentOfInertia;
    float angularVelocity;
    float mass;
    float width;
    float height;
    float rotation;
    Color colour;
    float force;
    float linearAccel;
    float angularAccel;
    private Array<PhysicsRect> childRects;
    private Boolean forceActing;

    public PhysicsRect(float x, float y, float width, float height, Color colour, float mass, float rotation)
    {
        position = new Vector2(x,y);
        velocity = new Vector2(0,0);
        angularVelocity = 0.0f;
        this.width = width;
        this.height = height;
        this.colour = colour;
        this.mass = mass;
        this.rotation = rotation;
        childRects = new Array<PhysicsRect>();
        COM = new Vector2();
        force = 0;
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
        force = 0;
        radial = new Vector2();
        linearAccel = 0;
        momentOfInertia = 0;
        forcePosition = Vector2.Zero ;
    }

    public void addChild(PhysicsRect rect)
    {
        childRects.add(rect);
    }


    public void centralForceOn(Boolean forward)
    {
        forceActing = true;
        forcePosition.x = position.x - width/2;
        forcePosition.y = 0;
        forcePosition.sub(COM);
        forcePosition.rotate(rotation);
        forcePosition.add(COM);
        force = forward ? 10000 : -10000;
    }

    public void rightForceOn()
    {
        forceActing = true;
        forcePosition.x = position.x + width/2;
        forcePosition.y = position.y + height/2;
        forcePosition.sub(COM);
        forcePosition.rotate(rotation);
        forcePosition.add(COM);
        force = 5000;
    }

    public void forceOff() {
        forceActing = false;
        force = 0;
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
        forcePosition.rotate(rotation);
        force = 5000;
    }

    private float determineMomentOfIntertia(Vector2 centerOfMass)
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

    private float determineLinearAcceleration(float force, float totalMass)
    {
        return force/totalMass;
    }

    private float determineAngularAcceleration(float I)
    {
        Vector2 r = radial;
        Vector2 f = new Vector2(force, 0);
        float t = r.crs(f);

        System.out.println("torque: " + t);

        float angularAccel = 0.0f;
        angularAccel = t / I;

        System.out.println("angular accel: " + angularAccel);

        return angularAccel;
    }

    public void update(float time)
    {
        System.out.println("Delta time: " + time);
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

        float linearAcceleration = determineLinearAcceleration(force, totalMass);

        //Determine Velocity
        linearAccel = linearAcceleration;
        velocity.x += linearAcceleration * time;
        //velocity.y += linearAcceleration * time;

        float momentOfInertia = determineMomentOfIntertia(COM);
        this.momentOfInertia = momentOfInertia;
        System.out.println("I: " + momentOfInertia);

        float angularAcceleration = determineAngularAcceleration(momentOfInertia);

        // Determine Angular Velocity
        angularVelocity += angularAcceleration * time;
        angularAccel = angularAcceleration;

        // update position
        COM.x += velocity.x * time;
        COM.y += velocity.x * time;

        System.out.println("AngularVel: " + angularVelocity);

        float rotationThisFrame =  angularVelocity  * time * (float)(180/Math.PI);
        rotation += rotationThisFrame;

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
    }
}

