package com.mygdx.projectTerminus;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
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
        for(Pair<RigidBody, RigidBody> p : bPhaseResults)
        {
            if(p.getLeft() == elasticBox || p.getRight() == elasticBox)
            {
                eBoxCurrentColor = eBoxHitColor;
                break;
            }
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
    private void doNarrowPhase(ArrayList<Pair<RigidBody, RigidBody>> bodies)
    {
        
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
        if (car.isForceOn(car.BACKWARD_FORCE) || car.isForceOn(car.FORWARD_FORCE))
        {
            arrowPosition = car.backForwardForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.rectLine(arrowPosition.x, arrowPosition.y, arrowPosition.x + 40, arrowPosition.y, 6);
            shapeRenderer.triangle(arrowPosition.x + 40, arrowPosition.y - 10,arrowPosition.x + 40, arrowPosition.y + 10, arrowPosition.x + 50, arrowPosition.y);
        }

        if (car.isForceOn(car.TURNING_LEFT_FORCE))
        {
            arrowPosition = car.turnLeftForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.rectLine(arrowPosition.x, arrowPosition.y, arrowPosition.x + 40, arrowPosition.y, 6);
            shapeRenderer.triangle(arrowPosition.x + 40, arrowPosition.y - 10,arrowPosition.x + 40, arrowPosition.y + 10, arrowPosition.x + 50, arrowPosition.y);
        }

        if (car.isForceOn(car.TURNING_RIGHT_FORCE))
        {
            arrowPosition = car.turnRightForceLocation();
            shapeRenderer.setColor(arrowColour);
            shapeRenderer.rectLine(arrowPosition.x, arrowPosition.y, arrowPosition.x + 40, arrowPosition.y, 6);
            shapeRenderer.triangle(arrowPosition.x + 40, arrowPosition.y - 10,arrowPosition.x + 40, arrowPosition.y + 10, arrowPosition.x + 50, arrowPosition.y);
        }

        /*if (car.isForceOn())
        {

            shapeRenderer.setColor(arrowColour);
            shapeRenderer.rectLine(arrowPosition.x, arrowPosition.y, arrowPosition.x + 40, arrowPosition.y, 6);
            shapeRenderer.triangle(arrowPosition.x + 40, arrowPosition.y - 10,arrowPosition.x + 40, arrowPosition.y + 10, arrowPosition.x + 50, arrowPosition.y);
        }*/

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
