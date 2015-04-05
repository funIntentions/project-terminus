package com.mygdx.projectTerminus;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;

/**
 * Created by Dan on 4/5/2015.
 */
public class ProjectTerminus implements Screen
{
    final Game game;
    private OrthographicCamera camera;
    private ShapeRenderer shapeRenderer;
    private PhysicsRect car;
    private PhysicsRect driver;
    private PhysicsRect tank;
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
    }

    @Override
    public void render(float deltaTime)
    {
        Gdx.gl.glClearColor( 0.2f,  0.2f, 0.2f, 1);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        if (running)
        {
            if (accelerating && (time + deltaTime > 2.0000f))
            {
                float difference = (time + deltaTime) - 2.0000f;
                car.update(deltaTime - difference);
                time += (deltaTime - difference);
                car.force = 0;
                accelerating = false;
            }
            else if (time + deltaTime >= 8.0000f)
            {
                float difference = (time + deltaTime) - 8.0000f;
                car.update(deltaTime - difference);
                time += (deltaTime - difference);
                running = false;
            }
            else
            {
                car.update(deltaTime);
                time += deltaTime;
            }
        }

        arrowPosition = car.forcePosition;

        camera.update();
        game.batch.setProjectionMatrix(camera.combined);
        shapeRenderer.setProjectionMatrix(camera.combined);

        shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
        shapeRenderer.setColor(1, 1, 0, 1);
        shapeRenderer.line(-Gdx.graphics.getWidth() / 2, 0, Gdx.graphics.getWidth() / 2, 0); // X Axis
        shapeRenderer.line(0, -Gdx.graphics.getHeight()/2, 0, Gdx.graphics.getHeight()/2); // Y Axis
        shapeRenderer.end();

        shapeRenderer.begin(ShapeRenderer.ShapeType.Filled); // draw car

        shapeRenderer.setColor(car.colour);
        shapeRenderer.identity();
        shapeRenderer.translate(car.position.x, car.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, car.rotation);
        shapeRenderer.rect(-car.width/2, -car.height/2, car.width, car.height); // Car
        shapeRenderer.rotate(0,0, 1, -car.rotation);
        shapeRenderer.translate(-car.position.x, -car.position.y, 0.f);

        shapeRenderer.setColor(driver.colour);
        shapeRenderer.translate(driver.position.x, driver.position.y, 0.f);
        shapeRenderer.rotate(0,0,1, driver.rotation);
        shapeRenderer.rect(-driver.width/2, -driver.height/2, driver.width, driver.height); // Driver
        shapeRenderer.rotate(0,0,1,-driver.rotation);
        shapeRenderer.translate(-driver.position.x, -driver.position.y, 0.f);

        shapeRenderer.setColor(tank.colour);
        shapeRenderer.translate(tank.position.x, tank.position.y, 0.f);
        shapeRenderer.rotate(0, 0, 1, tank.rotation);
        shapeRenderer.rect(-tank.width/2, -tank.height / 2, tank.width, tank.height); // Driver
        shapeRenderer.rotate(0,0,1,-tank.rotation);
        shapeRenderer.translate(-tank.position.x, -tank.position.y, 0.f);

        shapeRenderer.setColor(COMcolour);
        shapeRenderer.circle(car.COM.x, car.COM.y, 4);

        shapeRenderer.setColor(arrowColour);
        shapeRenderer.rectLine(arrowPosition.x, arrowPosition.y, arrowPosition.x + 40, arrowPosition.y, 6);
        shapeRenderer.triangle(arrowPosition.x + 40, arrowPosition.y - 10,arrowPosition.x + 40, arrowPosition.y + 10, arrowPosition.x + 50, arrowPosition.y);
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

        if (Gdx.input.isKeyJustPressed(Input.Keys.UP))
        {
            car.angularAccelerationOn();
        }

        if (Gdx.input.isKeyJustPressed(Input.Keys.DOWN))
        {
            car.angularAccelerationOff();
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