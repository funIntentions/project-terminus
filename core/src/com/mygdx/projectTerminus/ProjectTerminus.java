package com.mygdx.projectTerminus;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.math.Vector3;
import java.util.ArrayList;

/**
 * Created by Dan on 4/5/2015.
 */
public class ProjectTerminus implements Screen
{
    // For testing purposes
    private final Color eBoxUnmolestedColor = new Color(0.f, 0.f, 255.f, 1.f);
    private final Color eBoxHitColor = new Color(255.f, 0.f, 0.f, 1.f);
    private Color eBoxCurrentColor = eBoxUnmolestedColor;
    
    final Game game;
    private OrthographicCamera camera;
    private ShapeRenderer shapeRenderer;
    private PhysicsRect car;
    private PhysicsRect driver;
    private PhysicsRect tank;
    private PhysicsBox elasticBox;
    private ArrayList<RigidBody> bodies;
    private Color COMcolour;
    private Color arrowColour;
    private Vector2 arrowPosition;
    private Boolean running;
    private float time = 0;
    private Boolean accelerating;

    public ProjectTerminus(Game game)
    {
        this.game = game;

        shapeRenderer = new ShapeRenderer();

        camera = new OrthographicCamera();
        camera.setToOrtho(false, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
        camera.position.sub(Gdx.graphics.getWidth() / 2, Gdx.graphics.getHeight() / 2, 0);

        car = new PhysicsRect(-300, 0, 200, 100, new Color(0.5f,0.5f,0.5f,1f), 1000, 0);
        driver = new PhysicsRect(-240, 20, 40, 40, new Color(0.5f, 0f, 0f, 1f), 0, 0);
        tank = new PhysicsRect(-380, 0, 40, 80, new Color(0f, 0.5f, 0f, 1f), 0, 0);

        COMcolour = new Color(0,0,0.5f,1.0f);
        arrowColour = new Color(0.6f,0.6f,0f,1.0f);

        arrowPosition = new Vector2(0,0);

        running = false;
        accelerating = true;

        car.addChild(driver);
        car.addChild(tank);
                
        elasticBox = new PhysicsBox(100.f, new Vector2(170, 0), 0, 500.f,
                                    new Vector2(), 0.f, true);
        
        bodies = new ArrayList<RigidBody>();
        bodies.add(car);
        bodies.add(elasticBox);
    }

    /**
     * Detect and handle any collisions.
     */
    private void handleCollisions()
    {
        ArrayList<Pair<RigidBody, RigidBody>> bPhaseResults = doBroadPhase();
        ArrayList<CollisionInfo> nPhaseResults = doNarrowPhase(bPhaseResults);
        
        for(int i = 0; i < bPhaseResults.size(); i++)
        {
            Pair<RigidBody, RigidBody> curPair = bPhaseResults.get(i);
            CollisionInfo curInfo = nPhaseResults.get(i);
            
            // If we're not looking at the box or there wsa no collision, continue
            if(!(curPair.getLeft() == elasticBox || curPair.getRight() == elasticBox)) continue;
            else if(curInfo == null) continue;
            
            // If it got to here, then it was hit
            // Wow is this ever bad code...
            eBoxCurrentColor = eBoxHitColor;                
        }
    }
    
    // Does broadphase collision detection and returns an arraylist containing
    // pairs of possible collisions
    private ArrayList<Pair<RigidBody, RigidBody>> doBroadPhase()    
    {
        ArrayList<Pair<RigidBody, RigidBody>> possiblyColliding = new ArrayList<Pair<RigidBody, RigidBody>>();
        for(int b1Index = 0; b1Index < bodies.size(); b1Index++)
        {
            RigidBody body1 = bodies.get(b1Index);
            for(int b2Index = b1Index + 1; b2Index < bodies.size(); b2Index++)
            {
                RigidBody body2 = bodies.get(b2Index);
                double dist = body1.position.dst(body2.position);
                if(dist < (body1.getBoundingCircleRadius() + body2.getBoundingCircleRadius()))
                {
                    Pair<RigidBody, RigidBody> collPair = new Pair<RigidBody, RigidBody>(body1, body2);
                    possiblyColliding.add(collPair);
                }
            }
        }
        
        return possiblyColliding;
    }
    
    /**
     * Checks more thoroughly whether the given pairs collided.
     * @param bodies The bodies that may be colliding (as identified during
     * broadphase).
     */
    private ArrayList<CollisionInfo> doNarrowPhase(ArrayList<Pair<RigidBody, RigidBody>> bodies)
    {
        // Will contain the information about collisions for each pair, if any
        final ArrayList<CollisionInfo> collisionInfo = new ArrayList<CollisionInfo>();
        final int numPairs = bodies.size();
        boolean didCollide = false;
        
        for(int i = 0; i < numPairs; i++)
        {
            Pair<RigidBody, RigidBody> collPair = bodies.get(i);
            Vector2[] b1Vertices = collPair.getLeft().getVertices();
            Vector2[] b2Vertices = collPair.getRight().getVertices();
            
            // Use the separating axis theorem to check for collisions
            // We know that we're using boxes, so we can get away with checking
            // just 4 axes
            float x;
            Vector2 axes[] = new Vector2[4];
            // Calculate the edge vector and then find the normal (flip the slope and change the sign)
            axes[0] = new Vector2(b1Vertices[1]).sub(b1Vertices[0]);
            x = axes[0].x;
            axes[0].x = -axes[0].y;
            axes[0].y = x;
            axes[0].nor();
            
            axes[1] = new Vector2(b1Vertices[2]).sub(b1Vertices[1]);
            x = axes[1].x;
            axes[1].x = -axes[1].y;
            axes[1].y = x;
            axes[1].nor();
            
            axes[2] = new Vector2(b2Vertices[1]).sub(b2Vertices[0]);
            x = axes[2].x;
            axes[2].x = -axes[2].y;
            axes[2].y = x;
            axes[2].nor();
            
            axes[3] = new Vector2(b2Vertices[2]).sub(b2Vertices[1]);
            x = axes[3].x;
            axes[3].x = -axes[3].y;
            axes[3].y = x;
            axes[3].nor();
            
            int aIndex;
            for(aIndex = 0; aIndex < axes.length; aIndex++)
            {
                Pair<Vector2, Vector2> minMax1 = new Pair<Vector2, Vector2>();
                Pair<Vector2, Vector2> minMax2 = new Pair<Vector2, Vector2>();;
                
                Pair<Float, Float> proj1 = new Pair<Float, Float>();
                Pair<Float, Float> proj2 = new Pair<Float, Float>();
                
                getMinMax(collPair.getLeft(), axes[aIndex], minMax1, proj1);
                getMinMax(collPair.getRight(), axes[aIndex], minMax2, proj2);
                
                //System.out.println("Projection 1: " + proj1 + ", Projection 2: " + proj2);
                
                // If there's a gap between the projected vectors, then there was no collision
                if(proj1.getRight() < proj2.getLeft() || 
                   proj2.getRight() < proj1.getLeft())
                {
                    collisionInfo.add(i, null);
                    break;
                }
            }
            
            // If we looped through every axis and none of them had a gap, then
            // there was a collision
            if(aIndex == axes.length)
            {
                // Get the contact manifold
                
                // Add a dummy one for now; this is where we would get the manifold though
                CollisionInfo c = new CollisionInfo(0.f, null, null, null, null, null);
                collisionInfo.add(i, c);
            }
        }
        return collisionInfo;
    }
    
    /**
     * Gets the minimum and maximum points projected on a given axis.
     * @param rect      The rectangle for which to get the points.
     * @param axis      The axis along which the rectangle's points will be projected.
     * @param pointsOut A pair in which to store the points. 
     *                  Left is the minimum point, right the maximum.
     * @param projectionsOut A pair in which to store the projections. Left is the
     *                       minimum projection, right the maximum.
     */
    private void getMinMax(RigidBody rect, Vector2 axis,
                                             Pair<Vector2, Vector2> pointsOut,
                                             Pair<Float, Float> projectionsOut)
    {
        final Vector2[] vertices = rect.getVertices();
        final int numVertices = vertices.length;
        Vector2 minPoint = vertices[0];
        Vector2 maxPoint = vertices[0];
        float minProj = vertices[0].dot(axis);
        float maxProj = vertices[0].dot(axis);
        
        // Project each point on to the axis of projection and determine if it
        // should be the new maximum or minimum point
        for(int i = 1; i < numVertices; i++)
        {
            float dot = vertices[i].dot(axis);
            if(dot < minProj)
            {
                minPoint = vertices[i];
                minProj = dot;
            }
            else if(dot > maxProj)
            {
                maxPoint = vertices[i];
                maxProj = dot;
            }
        }
        
        pointsOut.setLeft(minPoint);
        pointsOut.setRight(maxPoint);
        
        projectionsOut.setLeft(minProj);
        projectionsOut.setRight(maxProj);
    }
    
    @Override
    public void render(float deltaTime)
    {
        Gdx.gl.glClearColor( 0.2f,  0.2f, 0.2f, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        car.update(deltaTime);
        handleCollisions();
        arrowPosition = car.forcePosition;

        camera.update();
        game.batch.setProjectionMatrix(camera.combined);
        shapeRenderer.setProjectionMatrix(camera.combined);

        shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        shapeRenderer.setColor(1, 1, 0, 1);
        shapeRenderer.line(-Gdx.graphics.getWidth() / 2, 0, Gdx.graphics.getWidth() / 2, 0); // X Axis
        shapeRenderer.line(0, -Gdx.graphics.getHeight() / 2, 0, Gdx.graphics.getHeight() / 2); // Y Axis
        shapeRenderer.end();

        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled); // draw car

        shapeRenderer.setColor(car.colour);
        shapeRenderer.identity();
        shapeRenderer.translate(car.position.x, car.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, car.rotation);
        shapeRenderer.rect(-car.width / 2, -car.height / 2, car.width, car.height); // Car
        shapeRenderer.rotate(0, 0, 1, -car.rotation);
        shapeRenderer.translate(-car.position.x, -car.position.y, 0.f);

        shapeRenderer.setColor(driver.colour);
        shapeRenderer.translate(driver.position.x, driver.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, driver.rotation);
        shapeRenderer.rect(-driver.width / 2, -driver.height / 2, driver.width, driver.height); // Driver
        shapeRenderer.rotate(0, 0, 1, -driver.rotation);
        shapeRenderer.translate(-driver.position.x, -driver.position.y, 0.f);

        shapeRenderer.setColor(tank.colour);
        shapeRenderer.translate(tank.position.x, tank.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, tank.rotation);
        shapeRenderer.rect(-tank.width / 2, -tank.height / 2, tank.width, tank.height); // Driver
        shapeRenderer.rotate(0, 0, 1, -tank.rotation);
        shapeRenderer.translate(-tank.position.x, -tank.position.y, 0.f);

        shapeRenderer.setColor(COMcolour);
        shapeRenderer.circle(car.COM.x, car.COM.y, 4);
        
        // Draw the elastic box
        shapeRenderer.setColor(eBoxCurrentColor);
        shapeRenderer.translate(elasticBox.position.x, elasticBox.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, elasticBox.rotation);
        shapeRenderer.rect(-elasticBox.sideLen / 2, -elasticBox.sideLen / 2,
                            elasticBox.sideLen, elasticBox.sideLen);
        shapeRenderer.rotate(0, 0, 1, -elasticBox.rotation);
        shapeRenderer.translate(-elasticBox.position.x, -elasticBox.position.y, 0.f);

        // Draw the force arrow if a force is currently being applied
        if (car.isForceOn(car.BACKWARD_FORCE))
        {
            arrowPosition = car.backForwardForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.translate(arrowPosition.x, arrowPosition.y, 0.f);
            shapeRenderer.rotate(0, 0, 1, -(180 - car.rotation));
            shapeRenderer.rectLine(0, 0, 40, 0, 6);
            shapeRenderer.triangle(40, -10, 40, 10, 50, 0);
            shapeRenderer.rotate(0, 0, 1, (180 - car.rotation));
            shapeRenderer.translate(-arrowPosition.x, -arrowPosition.y, 0.f);
        }

        if (car.isForceOn(car.FORWARD_FORCE))
        {
            arrowPosition = car.backForwardForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.translate(arrowPosition.x, arrowPosition.y, 0.f);
            shapeRenderer.rotate(0, 0, 1, car.rotation);
            shapeRenderer.rectLine(0, 0, 40, 0, 6);
            shapeRenderer.triangle(40, -10, 40, 10, 50, 0);
            shapeRenderer.rotate(0, 0, 1, -car.rotation);
            shapeRenderer.translate(-arrowPosition.x, -arrowPosition.y, 0.f);
        }

        if (car.isForceOn(car.TURNING_LEFT_FORCE))
        {
            arrowPosition = car.turnLeftForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.translate(arrowPosition.x, arrowPosition.y, 0.f);
            shapeRenderer.rotate(0, 0, 1, car.rotation);
            shapeRenderer.rectLine(0, 0, 40, 0, 6);
            shapeRenderer.triangle(40, -10, 40, 10,  50, 0);
            shapeRenderer.rotate(0, 0, 1, -car.rotation);
            shapeRenderer.translate(-arrowPosition.x, -arrowPosition.y, 0.f);
        }

        if (car.isForceOn(car.TURNING_RIGHT_FORCE))
        {
            arrowPosition = car.turnRightForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.translate(arrowPosition.x, arrowPosition.y, 0.f);
            shapeRenderer.rotate(0, 0, 1, car.rotation);
            shapeRenderer.rectLine(0, 0, 40, 0, 6);
            shapeRenderer.triangle(40, -10, 40, 10, 50, 0);
            shapeRenderer.rotate(0, 0, 1, -car.rotation);
            shapeRenderer.translate(-arrowPosition.x, -arrowPosition.y, 0.f);
        }

        shapeRenderer.end();
        game.batch.begin();

        game.font.drawMultiLine(game.batch, "Car Position: " + car.position + "\n" +
                "Tank Position: " + tank.position + "\n" +
                "Driver Position: " + driver.position + "\n\n" +
                "COM Position: " + car.COM + "\n\n" +
                "Moment of Inertia: " + car.momentOfInertia + "\n" +
                "Car + Tank + Driver = " + "( " + car.mass + ", " + tank.mass + ", " + driver.mass + " )" + "\n\n" +
                "Force: " + car.force + "\n" +
                "Time: " + time + "\n" +
                "v: " + car.velocity + "\n" +
                "a: " + car.linearAccel + "\n" +
                "theta: " +  (car.rotation * Math.PI/180) + "\n" +
                "omega: " + car.angularVelocity + "\n" +
                "alpha: " + car.angularAccel + "\n" +
                "Radial: " + car.radial + "\n", 100, Gdx.graphics.getHeight()/2.2f);
        game.batch.end();

        handleInput();
    }

    public void handleInput()
    {
        if (Gdx.input.isKeyJustPressed(Input.Keys.SPACE))
        {
            running = !running;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.W))
        {
            car.mass += 10;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.S))
        {
            car.mass -= 10;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.Q))
        {
            tank.mass += 10;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.A))
        {
            tank.mass -= 10;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.E))
        {
            driver.mass += 10;
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.D))
        {
            driver.mass -= 10;
        }

        if (Gdx.input.isKeyPressed(Input.Keys.UP))
        {
            car.centralForceOn(true);
        }
        else
        {
            car.centralForceOff(true);
        }

        if (Gdx.input.isKeyPressed(Input.Keys.DOWN))
        {
            car.centralForceOn(false);
        }
        else
        {
            car.centralForceOff(false);
        }

        if (Gdx.input.isKeyPressed(Input.Keys.RIGHT))
        {
            car.rightForceOn();
        }
        else
        {
            car.rightForceOff();
        }

        if (Gdx.input.isKeyPressed(Input.Keys.LEFT))
        {
            car.leftForceOn();
        }
        else
        {
            car.leftForceOff();
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.BACKSPACE))
        {
            car.reset(-300, 0, 200, 100, new Color(0.5f,0.5f,0.5f,1f), 1000, 0);
            driver.reset(-240, 20, 40, 40, new Color(0.5f, 0f, 0f, 1f), 0, 0);
            tank.reset(-380, 0, 40, 80, new Color(0f, 0.5f, 0f, 1f), 0, 0);
            time = 0;
            running = false;
            accelerating = true;
        }
    }

    @Override
    public void resize(int width, int height) {
    }

    @Override
    public void show() {
    }

    @Override
    public void hide() {
    }

    @Override
    public void pause() {
    }

    @Override
    public void resume() {
    }

    @Override
    public void dispose() {
    }
}
