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
    
    private ArrayList<Pair<Vector2, Vector2>> collidingEdges = new ArrayList<Pair<Vector2, Vector2>>();
    
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
        
        for(int i = 0; i < nPhaseResults.size(); i++)
        {
            CollisionInfo curInfo = nPhaseResults.get(i);
            
            // If we're not looking at the box or there wsa no collision, continue
            if(!(curInfo.bodies.getLeft() == elasticBox || curInfo.bodies.getRight() == elasticBox)) continue;            
            // If it got to here, then it was hit
            // Wow is this ever bad code...
            eBoxCurrentColor = eBoxHitColor;
            System.out.println("Collision Info: " + curInfo);
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
            int minAxisIdx = 0;
            float minTranslation = Float.MAX_VALUE;
            for(aIndex = 0; aIndex < axes.length; aIndex++)
            {
                Pair<Vector2, Vector2> minMax1 = new Pair<Vector2, Vector2>();
                Pair<Vector2, Vector2> minMax2 = new Pair<Vector2, Vector2>();;
                
                Pair<Float, Float> proj1 = new Pair<Float, Float>();
                Pair<Float, Float> proj2 = new Pair<Float, Float>();
                
                getMinMax(collPair.getLeft(), axes[aIndex], minMax1, proj1);
                getMinMax(collPair.getRight(), axes[aIndex], minMax2, proj2);

                float translation;
                if (proj1.getRight() > proj2.getLeft())
                {
                    translation = proj1.getRight() - proj2.getLeft();
                    if (translation < minTranslation) minTranslation = translation;
                    minAxisIdx = aIndex;
                }
                else if(proj2.getRight() > proj1.getLeft())
                {
                    translation = proj2.getRight() - proj1.getLeft();
                    if (translation < minTranslation) minTranslation = translation;
                    minAxisIdx = aIndex;
                }
                
                // If there's a gap between the projected vectors, then there was no collision
                if(proj1.getRight() < proj2.getLeft() || 
                   proj2.getRight() < proj1.getLeft())
                {
                    break;
                }
            }
            
            // If we looped through every axis and none of them had a gap, then
            // there was a collision
            if(aIndex == axes.length)
            {
                // Flip the normal if it doesn't point in the direction of the collision
                Vector2 obj1To2 = new Vector2(collPair.getRight().position)
                                             .sub(collPair.getLeft().position).nor();
                if(obj1To2.dot(axes[minAxisIdx]) < 0)
                        axes[minAxisIdx].scl(-1);
                
                // Determine the referent and incident faces
                // We always assume that the referent face belongs to the first object in the collision pair
                Pair<Vector2, Vector2> bestEdge1 = getBestEdge(new Vector2(axes[minAxisIdx]), collPair.getLeft());
                Pair<Vector2, Vector2> bestEdge2 = getBestEdge(new Vector2(axes[minAxisIdx]).scl(-1), collPair.getRight());
                ArrayList<Vector2> collisionPoints = findCollisionPoints(axes[minAxisIdx], bestEdge1, bestEdge2);

                CollisionInfo c = new CollisionInfo(collPair, collisionPoints, minTranslation, axes[minAxisIdx]);
                collisionInfo.add(i, c);
                
                collidingEdges = new ArrayList<Pair<Vector2, Vector2>>();
                collidingEdges.add(bestEdge1);
                collidingEdges.add(bestEdge2);
            }
        }
        return collisionInfo;
    }

    // clips the line segment points v1, v2
    // if they are past o along n
    ArrayList<Vector2> clip(Vector2 v1, Vector2 v2, Vector2 n, double o)
    {
        ArrayList<Vector2> cp = new ArrayList<Vector2>();
        double d1 = n.dot(v1) - o;
        double d2 = n.dot(v2) - o;
        // if either point is past o along n
        // then we can keep the point
        if (d1 >= 0.0) cp.add(v1);
        if (d2 >= 0.0) cp.add(v2);
        // finally we need to check if they
        // are on opposing sides so that we can
        // compute the correct point
        if (d1 * d2 < 0.0) {
            // if they are on different sides of the
            // offset, d1 and d2 will be a (+) * (-)
            // and will yield a (-) and therefore be
            // less than zero
            // get the vector for the edge we are clipping
            Vector2 e = new Vector2(v2.x - v1.x, v2.y - v1.y);
            // compute the location along e
            double u = d1 / (d1 - d2);
            e.x *= u;
            e.y *= u;
            e = e.add(v1);
            // add the point
            cp.add(e);
        }

        return cp;
    }

    private ArrayList<Vector2> findCollisionPoints(Vector2 normal, Pair<Vector2, Vector2> edge1, Pair<Vector2, Vector2> edge2)
    {
        Pair<Vector2, Vector2> refEdge, incEdge;
        Vector2 edge1Vector = new Vector2(edge1.getRight().x - edge1.getLeft().x, edge1.getRight().y - edge1.getLeft().y);
        Vector2 edge2Vector = new Vector2(edge2.getRight().x - edge2.getLeft().x, edge2.getRight().y - edge2.getLeft().y);
        Boolean flip = false;

        // determine reference and incident edges
        if (Math.abs(edge1Vector.dot(normal)) <= Math.abs(edge2Vector.dot(normal)))
        {
            refEdge = edge1;
            incEdge = edge2;
        }
        else
        {
            incEdge = edge1;
            refEdge = edge2;
            flip = true;
        }

        Vector2 refVector = new Vector2(refEdge.getRight().x - refEdge.getLeft().x, refEdge.getRight().y - refEdge.getLeft().y);
        refVector.nor();

        double offset1 = refVector.dot(refEdge.getLeft());
        // clip incident edge by the first vertex of the reference edge
        ArrayList<Vector2> clippedPoints = clip(incEdge.getLeft(), incEdge.getRight(), refVector, offset1);
        if (clippedPoints.size() < 2) return null; // failure

        double offset2 = refVector.dot(refEdge.getRight());
        // clip incident edge by the second vertex of reference edge in opposite direction
        clippedPoints = clip(clippedPoints.get(0), clippedPoints.get(1), new Vector2(-refVector.x, -refVector.y), -offset2);
        if (clippedPoints.size() < 2) return null; // failure

        // clip points past the reference edge along the reference edge's normal
        Vector2 refEdgeNormal = new Vector2(-refVector.y, refVector.x);
        if (flip) refEdgeNormal.scl(-1);

        double offset3 = refEdgeNormal.dot(refEdge.getRight());
        ArrayList<Vector2> pointsToRemove = new ArrayList<Vector2>();

        for (Vector2 point : clippedPoints)
        {
            if (refEdgeNormal.dot(point) - offset3 < 0.0)
                pointsToRemove.add(point);
        }

        //clippedPoints.removeAll(pointsToRemove);
        return clippedPoints;
    }
    
    private Pair<Vector2, Vector2> getBestEdge(Vector2 normal, RigidBody rect)
    {
        Vector2[] vertices = rect.getVertices();
        
        // Find the vertex furthest along the normal
        float maxProj = vertices[0].dot(normal);
        int furthestIndex = 0;
        for(int i = 1; i < vertices.length; i++)
        {
            float dot = vertices[i].dot(normal);
            if(dot > maxProj)
            {
                maxProj = dot;
                furthestIndex = i;
            }
        }
        
        // Get the left and right edges containing this vertex
        Vector2 furthest = vertices[furthestIndex];
        Vector2 leftNeighbour = furthestIndex == 0 ? vertices[vertices.length - 1] : vertices[furthestIndex - 1];
        Vector2 rightNeighbour = furthestIndex == vertices.length - 1 ? vertices[0]: vertices[furthestIndex + 1];
        
        Vector2 leftEdge = new Vector2(furthest).sub(leftNeighbour).nor();
        Vector2 rightEdge = new Vector2(furthest).sub(rightNeighbour).nor();
        
        float lDot = leftEdge.dot(normal);
        float rDot = rightEdge.dot(normal);

        return lDot < rDot ? new Pair<Vector2, Vector2>(leftNeighbour, furthest)
                           : new Pair<Vector2, Vector2>(furthest, rightNeighbour);
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
    
    private void collide(final RigidBody b1, final RigidBody b2,
                         final float elasticity)
    {
//        Vector3 unitNormal = new Vector3(1, 0, 0);
//        Vector3 P = new Vector3(0,0,0);
//        P.x = circle2.position.x - 20;
//        P.y = circle1.position.y + ((circle2.position.y - circle1.position.y)/2.0f);
//
//        Vector3 r1 = new Vector3((P.x - circle1.position.x), (P.y - circle1.position.y), 0);
//        Vector3 r2 = new Vector3((P.x - circle2.position.x), (P.y - circle2.position.y), 0);
//
//        initialOrbital1 = r1.y * circle1.mass * circle1.velocity.x;
//        initialOrbital2 = r2.y * circle2.mass * circle2.velocity.x;
//
//        float momentOfInertia1 = (circle1.mass * (float)(Math.pow(40, 2) + Math.pow(40, 2)))/12.0f;
//        float momentOfInertia2 = (circle2.mass * (float)(Math.pow(40, 2) + Math.pow(40, 2)))/ 12.0f;
//
//        float vR = circle1.velocity.x - circle2.velocity.x;
//
//        // calculate impulse J
//        float coefficient = -vR * (e + 1.0f);
//        float inverseMasses = (1.0f/circle1.mass) + (1.0f/circle2.mass);
//
//        Vector3 elementOne = new Vector3(r1);
//        elementOne = elementOne.crs(unitNormal);
//        elementOne.x /= momentOfInertia1;
//        elementOne.y /= momentOfInertia1;
//        elementOne.z /= momentOfInertia1;
//        elementOne = elementOne.crs(r1);
//        float componentOne = unitNormal.dot(elementOne); //Vector3.dot(unitNormal.x, unitNormal.y, unitNormal.z, elementOne.x, elementOne.y, elementOne.z);
//
//        Vector3 elementTwo = new Vector3(r2);
//        elementTwo = elementTwo.crs(unitNormal);
//        elementTwo.x /= momentOfInertia2;
//        elementTwo.y /= momentOfInertia2;
//        elementTwo.z /= momentOfInertia2;
//        elementTwo = elementTwo.crs(r2);
//        float componentTwo = unitNormal.dot(elementTwo); //Vector3.dot(unitNormal.x, unitNormal.y, unitNormal.z, elementTwo.x, elementTwo.y, elementTwo.z);
//
//        float denominator = inverseMasses + componentOne + componentTwo;
//
//        float J = coefficient * (1.0f/denominator);
//
//        float Uf = J/circle1.mass + circle1.velocity.x;
//        float Vf = -J/circle2.mass + circle2.velocity.x;
//
//        pix = (circle1.mass * initialVelocityU.x) + (circle2.mass * initialVelocityV.x);
//        pfx = (circle1.mass * circle1.velocity.x) + (circle2.mass * circle2.velocity.x);
//        pfy = (circle1.mass * circle1.velocity.y) + (circle2.mass * circle2.velocity.y);
//
//        impulse = J;
//        colisionCount++;
//
//        circle1.velocity.x = Uf * unitNormal.x;
//        circle1.velocity.y = Uf * unitNormal.y;
//        circle2.velocity.x = Vf * unitNormal.x;
//        circle2.velocity.y = Vf * unitNormal.y;
//
//        Vector3 angularVelocity1 = new Vector3(unitNormal);
//        angularVelocity1.x = angularVelocity1.x * J;
//        angularVelocity1.y = angularVelocity1.y * J;
//        angularVelocity1.z = angularVelocity1.z * J;
//        angularVelocity1 = (new Vector3(r1)).crs(angularVelocity1);
//        rect1W = angularVelocity1.z * (1.0f/momentOfInertia1) * (float)(180.0f/Math.PI);
//
//        Vector3 angularVelocity2 = new Vector3(unitNormal);
//        angularVelocity2.x = angularVelocity2.x * -J;
//        angularVelocity2.y = angularVelocity2.y * -J;
//        angularVelocity2.z = angularVelocity2.z * -J;
//        angularVelocity2 = (new Vector3(r2)).crs(angularVelocity2);
//        rect2W = angularVelocity2.z * (1.0f/momentOfInertia2) * (float)(180.0f/Math.PI);
//
//        circle1KEf = (0.5f) * circle1.mass * (float)Math.pow(circle1.velocity.x, 2);
//        circle2KEf = (0.5f) * circle2.mass * (float)Math.pow(circle2.velocity.x, 2);
//
//        rect1RKEf = (0.5f) * momentOfInertia1 * (float)Math.pow(rect1W * (Math.PI/180.0f), 2);
//        rect2RKEf = (0.5f) * momentOfInertia2 * (float)Math.pow(rect2W * (Math.PI/180.0f), 2);
//
//        finalOrbital1 = r1.y * circle1.mass * circle1.velocity.len();
//        finalOrbital2 = r2.y * circle2.mass * circle2.velocity.len();
//        finalRotational1 = momentOfInertia1 * (float)(rect1W * (Math.PI/180.0f));
//        finalRotational2 = momentOfInertia2 * (float)(rect2W * (Math.PI/180.0f));
//
//        collided = true;
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
        
        for(Pair<Vector2, Vector2> edge: collidingEdges)
        {
            Vector2 p1 = edge.getLeft();
            Vector2 p2 = edge.getRight();
            shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
            shapeRenderer.setColor(1, 1, 0, 1);
            shapeRenderer.line(p1.x, p1.y, p2.x, p2.y); // X Axis
            shapeRenderer.end();
        }
        
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
