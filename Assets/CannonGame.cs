using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VerletPoint
{
    public Vector3 pos;
    public Vector3 prevPos;
    public Vector3 vel;
    public Vector3 acc;
    public bool isFixed;
    public bool isDrawn;


    Vector3 externalVel = new Vector3();

    public VerletPoint(Vector3 _pos, Vector3 _vel, bool _isFixed, bool _isDrawn)
    {
        pos = _pos;
        prevPos = _pos - _vel*Time.deltaTime;
        acc = new Vector3();
        isFixed = _isFixed;
        isDrawn = _isDrawn;
    }

    public void updatePosition( float deltaTime, float damping )
    {
        acc = new Vector3(0, -CannonGame.gravity*0.0f, 0);
        vel = pos - prevPos;
        prevPos = pos;
        pos = pos + (vel + externalVel)* damping + acc * deltaTime * deltaTime;
        externalVel = new Vector3();
    }

    public void addExternalVelocity(Vector3 addedVel)
    {
        externalVel += addedVel;
    }

    // Doesnt affect velocity;
    public void cheatDisplace(Vector3 val)
    {
        pos += val;
        prevPos += val;
    }
}

public class VerletConstraint
{
    public float minDist;
    public float maxDist;
    public VerletPoint p0;
    public VerletPoint p1;

    public VerletConstraint(VerletPoint _p0, VerletPoint _p1, float _minDist, float _maxDist)
    {
        p0 = _p0;
        p1 = _p1;
        minDist = _minDist;
        maxDist = _maxDist;
    } 

    // Call after movement to 'fix' points to maintain constraint
    public void resolve()
    {
        
        Vector3 v01 = p1.pos - p0.pos;
        float dist = v01.magnitude;
        float delta = 0;
        if (dist < minDist -0.01f)
        {
            delta = 0.5f * (minDist - dist) / dist;
        }
        else if (dist > maxDist + 0.01f)
        {
            delta = 0.5f * (maxDist - dist) / dist;
        }
        else
        {
            return;
        }

        //v01.Normalize();

        if (!p0.isFixed) p0.pos = p0.pos + delta * v01;
        if (!p1.isFixed) p1.pos = p1.pos - delta * v01;
        
    }
}

public class VerletTurkey
{
    VerletPoint anchor;
    public List<VerletPoint> points = new List<VerletPoint>();
    public List<VerletConstraint> constraints = new List<VerletConstraint>();
    float xMinBound;
    float xMaxBound;
    public float lastJumpTime = 0.0f;
    public bool inAir = false;

    static float turkeySpeed = 0.0f;

    int bodyVertexCount;
    int legVertexCount;
    int lLegStartIdx;
    int rLegStartIdx;

    bool facingLeft = true;

    public VerletTurkey( float scale, float _xMinBound, float _xMaxBound )
    {
        int sign = (Random.Range(0, 2) == 0) ? -1 : 1;
        Vector3 vel = new Vector3( sign*turkeySpeed, 0.0f );
        generateGeometry(scale, vel);
        //if (sign == 1)  flip();

        xMinBound = _xMinBound;
        xMaxBound = _xMaxBound;
    }

    private void generateGeometry( float scale, Vector3 vel )
    {
        float minX = CannonGame.wallWidth + CannonGame.wallStart + scale * 1.5f;
        float maxX = CannonGame.terrainStart - scale;
        Vector3 anchorPos = new Vector3(Random.Range(minX, maxX), scale * 1.65f, 0);
        anchor = new VerletPoint(anchorPos, vel, true, false);

        //temp storage
        Vector3 p;
        VerletPoint vP;
        VerletConstraint vC;

        //create round part of body
        float theta = Mathf.PI * 0.75f;
        float thetaEnd = -Mathf.PI * 0.5f;
        float delta = (thetaEnd - theta) / 9.0f;
        while (theta >= thetaEnd)
        {
            p = new Vector3(scale * Mathf.Cos(theta) + anchor.pos.x, scale * Mathf.Sin(theta) + anchor.pos.y);
            vP = new VerletPoint(p, vel, false, true);
            points.Add(vP);

            vC = new VerletConstraint(vP, anchor, scale - scale * 0.05f, scale + scale * 0.05f);
            constraints.Add(vC);
            theta += delta;
        }

        //Create flat bottom of body
        float dist = scale / 3;
        VerletPoint last = points[points.Count - 1];
        rLegStartIdx = points.Count;
        lLegStartIdx = points.Count + 1;
        for (int i = 0; i < 3; i++)
        {
            p = last.pos;
            p.x -= dist;
            vP = new VerletPoint(p, vel, false, true);
            points.Add(vP);

            vC = new VerletConstraint(vP, last, dist, dist);
            constraints.Add(vC);
            last = vP;
        }

        //Create chest 
        p = last.pos;
        p.x -= scale * 0.55f;
        p.y += scale * 0.75f;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);

        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        // Create left part of neck
        p = last.pos;
        dist = scale * 0.55f;
        p.y += dist;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);

        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        // Create wottle
        p = last.pos;
        p.x -= 0.075f * scale;
        p.y -= 0.65f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        p = last.pos;
        p.x -= 0.075f * scale;
        p.y += 0.675f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        // Create beak
        p = last.pos;
        p.x -= 0.25f * scale;
        p.y += 0.075f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;
        p = last.pos;
        p.x += 0.35f * scale;
        p.y += 0.25f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;


        // Top of head
        p = last.pos;
        p.x += 0.15f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        // Right of neck
        p = last.pos;
        p.x += 0.05f * scale;
        p.y -= 0.55f * scale;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        vC = new VerletConstraint(points[0], vP, dist, dist);
        constraints.Add(vC);

        bodyVertexCount = points.Count;
        // Left leg;
        last = points[lLegStartIdx];
        p = last.pos;
        p.y -= scale * 0.35f;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        p = last.pos;
        p.y -= scale * 0.35f;
        p.x -= scale * 0.2f;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        vC = new VerletConstraint(points[lLegStartIdx], vP, dist, dist);
        constraints.Add(vC);
        last = vP;

        dist = (points[lLegStartIdx].pos - vP.pos).magnitude;
        vC = new VerletConstraint(points[lLegStartIdx], vP, dist, dist);
        constraints.Add(vC);
        legVertexCount = points.Count - bodyVertexCount;

        // Right leg;
        last = points[rLegStartIdx];
        p = last.pos;
        p.y -= scale * 0.35f;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        p = last.pos;
        p.y -= scale * 0.35f;
        p.x -= scale * 0.2f;
        dist = (last.pos - p).magnitude;
        vP = new VerletPoint(p, vel, false, true);
        points.Add(vP);
        vC = new VerletConstraint(vP, last, dist, dist);
        constraints.Add(vC);
        last = vP;

        dist = (points[lLegStartIdx].pos - vP.pos).magnitude;
        vC = new VerletConstraint(points[rLegStartIdx], vP, dist, dist);
        constraints.Add(vC);
    }

    public static void setInitialTurkeySpeed(float speed)
    {
        turkeySpeed = speed;
    }

    public void move( float delta )
    {
        if (inAir) anchor.addExternalVelocity(new Vector3(0, -CannonGame.gravity * delta *delta/ 2.5f, 0));
        anchor.updatePosition(delta, 1.0f);
        foreach( VerletPoint p in points )
        {
            if (inAir) p.addExternalVelocity(new Vector3(0, -CannonGame.gravity * delta *delta /2.5f, 0));
            p.updatePosition( delta, 1.0f );
            if (p.pos.x <= xMinBound)//|| (p.pos.x >= xMaxBound ))
            {
                //displace(new Vector3(p.pos.x - xMinBound, 0.0f));
                flipVelocities();
            }
            else if ((p.pos.x >= xMaxBound))
            {
                //displace(new Vector3(p.pos.x - xMaxBound, 0.0f));
                flipVelocities();
            }
        }

        foreach (VerletConstraint c in constraints)
        {
            c.resolve();
        }
    }

    public void displace(Vector3 val)
    {
        anchor.cheatDisplace(val);
        foreach (VerletPoint p in points) p.cheatDisplace(val);
    }

    public void addExternalVelocity( Vector3 val )
    {
        anchor.addExternalVelocity(val);
        foreach (VerletPoint p in points) p.addExternalVelocity(val);
    }

    public void draw()
    {
        GL.Begin(GL.LINE_STRIP);
        int firstDrawn = -1;
        for (int i = 0; i < bodyVertexCount; i++)
        {
            if (points[i].isDrawn)
            {
                firstDrawn = (firstDrawn < 0) ? i : firstDrawn;
                GL.Vertex(points[i].pos);
            }
        }
        GL.Vertex(points[firstDrawn].pos);
        GL.End();

        GL.Begin(GL.LINE_STRIP);
        GL.Vertex(points[lLegStartIdx].pos);
        for (int i = bodyVertexCount; i < bodyVertexCount + legVertexCount; i++)
        {
            GL.Vertex(points[i].pos);
        }
        GL.End();

        GL.Begin(GL.LINE_STRIP);
        GL.Vertex(points[rLegStartIdx].pos);
        for (int i = bodyVertexCount + legVertexCount; i < bodyVertexCount + legVertexCount*2; i++)
        {
            GL.Vertex(points[i].pos);
        }
        GL.End();

    }

    //Flip velocity and position over anchor's x
    public void flip()
    {
        foreach (VerletPoint p in points)
        {
            p.prevPos.x = 2 * anchor.pos.x - p.prevPos.x;
            p.pos.x = 2 * anchor.pos.x - p.pos.x;
            facingLeft = !facingLeft;
        }
    }

    public void flipVelocities()
    {

        foreach (VerletPoint p in points)
        {
            p.addExternalVelocity( new Vector3(-2 * p.vel.x, 0.0f, 0.0f ) );
        }
        anchor.addExternalVelocity(new Vector3(-2 * anchor.vel.x, 0.0f, 0.0f));
    }

    public void jump( float jumpSpeed )
    {
        inAir = true;

        foreach (VerletPoint p in points)
        {
            p.addExternalVelocity(new Vector3(0, jumpSpeed * Time.deltaTime, 0));
        }
        anchor.addExternalVelocity(new Vector3(0, jumpSpeed * Time.deltaTime, 0));
    }

    public void ground()
    {
        inAir = false;
        foreach (VerletPoint p in points)
        {
            p.addExternalVelocity(new Vector3(0, -p.vel.y, 0));
        }
        anchor.addExternalVelocity(new Vector3(0, -anchor.vel.y, 0));
    }
}

public class Cannonball
{
    public const float mass = 1;
    public Vector3 centre;
    public Vector3 velocity;
    public float radius;
    public bool isGrounded = false;

    public Cannonball( Vector3 _centre, Vector3 _velocity, float _radius )
    {
        centre   = _centre;
        velocity = _velocity;
        radius   = _radius;
    }

    public Cannonball() { }

    public Cannonball( Cannonball other )
    {
        this.centre = other.centre;
        this.velocity = other.velocity;
        this.radius = other.radius;
        this.isGrounded = other.isGrounded;
    }
}

public class Rectangle
{
    public Vector3 a;
    public Vector3 b;
    public Vector3 c;
    public Vector3 d;
}


public class CannonGame : MonoBehaviour
{
    // Draws a Triangle, a Quad and a line
    // with different colors

    List<Vector3> terrain = new List<Vector3>();
    List<Cannonball> cannonballs = new List<Cannonball>();
    List<Vector3> cloudVerts = new List<Vector3>();
    List<VerletTurkey> turkeys = new List<VerletTurkey>();

    public float turkeySpeed;
    public float turkeyJumpSpeed;

    public static float wallStart = 0.0f;
    public static float wallWidth = 0.025f;
    public static float terrainStart = 0.33f;
    public float wallHeight;
    public float terrainEnd;
    public float spikiness;
    public int detailLevel;


    public float cannonRadius;
    public float cannonWidth;
    public float cannonLength;
    public float cannonXPos;
    public float cannonSpeed;
    private float cannonAngle = -Mathf.PI / 4;

    public float ballInitialSpeed;
    public float ballMinSpeed;
    [Range(0.5f, 0.95f)]
    public float ballBounceRestitution;

    public static float gravity = 6.0f;
    public float maxWindMag;

    private float currWindForce = 0.0f;
    private float mountainTopY = 0.0f;
    private float timeLastWindchange = 0.0f;

    private Vector3 cloudVelocity;
    public Vector3 iniCloudPos;
    public float simCloudMass;
    private Vector3 cloudPos;

    private void Start()
    {
        generateTerrain(detailLevel, spikiness);
        cloudPos = iniCloudPos;
        VerletTurkey.setInitialTurkeySpeed(turkeySpeed);
        turkeys.Add(new VerletTurkey(0.015f, wallWidth+wallStart, terrainStart));
        turkeys.Add(new VerletTurkey(0.015f, wallWidth+wallStart, terrainStart));
        turkeys.Add(new VerletTurkey(0.015f, wallWidth+wallStart, terrainStart));
        turkeys.Add(new VerletTurkey(0.015f, wallWidth+wallStart, terrainStart));
        turkeys.Add(new VerletTurkey(0.015f, wallWidth+wallStart, terrainStart));
    }

    private void Update()
    {
        controlCannon();
        simulateCannonballs();
        simulateWind();
        simulateCloud();
        simulateTurkeys();
    }

    void OnPostRender()
    {
        GL.PushMatrix();
        GL.LoadOrtho();

        GL.Clear(true, true, Color.white);

        GL.Begin(GL.QUADS); // Quad
        GL.Color(new Color(0.25f, 0.25f, 0.25f));
        GL.Vertex3(0.0f, wallHeight, 0.0f);
        GL.Vertex3(0.0f, 0.0f, 0.0f);
        GL.Vertex3(wallWidth, 0.0f, 0.0f);
        GL.Vertex3(wallWidth, wallHeight, 0.0f);
        GL.End();

        drawTerrain();
        drawCloud();
        drawCannon();
        drawCannonballs();
        drawTurkeys();


        GL.PopMatrix();
    }

    void generateTerrain(int granularity, float offCoeff)
    {
        if (granularity == 0)
        {
            return;
        }
        mountainTopY = 0.0f;
        // Genearate Initial Mountain
        if (terrain.Count <= 0)
        {
            terrain.Add(new Vector3(terrainStart, 0.0f, 0.0f));

            float diff = terrainEnd - terrainStart;
            terrain.Add(new Vector3(terrainStart + diff * 0.375f, 0.45f, 0.0f));
            terrain.Add(new Vector3(terrainStart + diff * 0.625f, 0.45f, 0.0f));
            mountainTopY = 0.5f;

            terrain.Add(new Vector3(terrainEnd, 0.0f, 0.0f));
        }
        else
        {
            List<Vector3> newTerrain = new List<Vector3>(terrain.Count * 2 - 1);
            for (int i = 0; i < terrain.Count - 1; i++)
            {
                Vector3 p0 = terrain[i];
                Vector3 p1 = terrain[i + 1];
                float len = (p0 - p1).magnitude;

                Vector3 mid = (p0 + p1) * 0.5f;
                float off = offCoeff * Random.Range(-len, len);
                newTerrain.Add(terrain[i]);
                mountainTopY = (terrain[i].y > mountainTopY) ? terrain[i].y : mountainTopY;

                Vector3 nP = mid + Vector3.up * off;
                newTerrain.Add(nP);

                mountainTopY = (nP.y > mountainTopY) ? nP.y : mountainTopY;
            }

            newTerrain.Add(terrain[terrain.Count - 1]);
            terrain = newTerrain;
        }

        generateTerrain(granularity - 1, offCoeff);
    }

    void drawTerrain()
    {
        GL.Begin(GL.LINE_STRIP);
        for (int i = 0; i < terrain.Count; i++)
        {
            GL.Vertex(terrain[i]);
        }
        GL.End();
    }

    void drawCircle(float radius, Vector3 centre)
    {
        GL.Begin(GL.TRIANGLES);
        float delta = 0.01f;
        float theta = 0.0f;
        while (theta <= 2 * Mathf.PI)
        {
            GL.Vertex(centre);
            GL.Vertex3(radius * Mathf.Cos(theta) + centre.x, radius * Mathf.Sin(theta) + centre.y, centre.z);
            theta += delta;
            GL.Vertex3(radius * Mathf.Cos(theta) + centre.x, radius * Mathf.Sin(theta) + centre.y, centre.z);
        }
        GL.End();
    }

    void drawCannon()
    {
        drawCircle(cannonRadius, new Vector3(cannonXPos, cannonRadius, 0.0f));

        Vector3 cannonPos = new Vector3(cannonXPos, (2 * cannonRadius - cannonWidth / 2));
        GL.Begin(GL.QUADS);
        {
            Vector3 lT = new Vector3(-cannonLength, cannonWidth / 2.0f);
            Vector3 lB = new Vector3(-cannonLength, -cannonWidth / 2.0f);
            Vector3 rB = new Vector3(0.0f, -cannonWidth / 2.0f);
            Vector3 rT = new Vector3(0.0f, cannonWidth / 2.0f);
            // Rotate in Cannon Space 
            Vector3 aimVector = new Vector3(Mathf.Cos(cannonAngle), Mathf.Sin(cannonAngle));
            Matrix4x4 rot = Matrix4x4.Rotate(Quaternion.FromToRotation(Vector3.right, aimVector));
            lT = rot * lT;
            lB = rot * lB;
            rB = rot * rB;
            rT = rot * rT;

            lT += cannonPos;
            lB += cannonPos;
            rB += cannonPos;
            rT += cannonPos;

            GL.Vertex(lT);
            GL.Vertex(lB);
            GL.Vertex(rB);
            GL.Vertex(rT);
        }
        GL.End();
    }

    void controlCannon()
    {
        // Rotate Cannon
        cannonAngle -= (Input.GetAxis("Vertical")) * cannonSpeed * Time.deltaTime;
        cannonAngle = Mathf.Clamp(cannonAngle, -Mathf.PI / 2, 0.0f);

        // Shoot
        if (Input.GetKeyDown("space"))
        {
            Debug.Log("SPACE");
            Vector3 aimVector = new Vector3(-Mathf.Cos(cannonAngle), -Mathf.Sin(cannonAngle));
            Vector3 velocity = aimVector.normalized;
            velocity *= ballInitialSpeed;
            Vector3 startPos = new Vector3(cannonXPos, (2 * cannonRadius - cannonWidth / 2));
            startPos += aimVector * cannonLength;
            cannonballs.Add(new Cannonball(startPos, velocity, 1.05f * cannonWidth / 2.0f));
        }
    }

    void simulateCannonballs()
    {

        for (int i = 0; i < cannonballs.Count; i++)
        {
            Cannonball curr = cannonballs[i];
            if (intersectCannonballWithScene(ref curr))
            {
                cannonballs.RemoveAt(i);
                i--;
                continue;
            }
            if (curr.velocity.magnitude <= ballMinSpeed)
            {
                cannonballs.RemoveAt(i);
                i--;
                continue;
            }

            float locGrav = (curr.isGrounded) ? 0.0f : gravity;
            curr.velocity.y -= locGrav * Time.deltaTime;
            if (curr.centre.y >= mountainTopY)
            {
                curr.velocity.x += (currWindForce / Cannonball.mass) * Time.deltaTime;
            }
            curr.centre += curr.velocity * Time.deltaTime;

        }
    }

    Cannonball cannonballNextFrame(Cannonball b)
    {
        Cannonball next = new Cannonball();
        next.velocity = b.velocity;
        next.velocity.y = next.velocity.y - gravity * Time.deltaTime;
        if (b.centre.y >= mountainTopY)
        {
            next.velocity.x = next.velocity.x + (currWindForce / Cannonball.mass) * Time.deltaTime;
        }
        next.centre = b.centre + b.velocity * Time.deltaTime;
        next.radius = b.radius;

        return next;
    }

    void drawCannonballs()
    {
        for (int i = 0; i < cannonballs.Count; i++)
        {
            drawCircle(cannonballs[i].radius, cannonballs[i].centre);
        }
    }

    void simulateWind()
    {
        float currTime = Time.time;
        if (currTime - timeLastWindchange >= 0.5f)
        {
            currWindForce = Random.Range(-maxWindMag, maxWindMag);
            timeLastWindchange = currTime;
        }
    }

    void drawCloud()
    {
        GL.Begin(GL.TRIANGLES);
        GL.Color(Color.white);
        for (int i = 0; i < cloudVerts.Count - 1; i++)
        {
            Vector3 v0 = cloudVerts[i];
            Vector3 v1 = cloudPos;
            Vector3 v2 = cloudVerts[i + 1];

            GL.Vertex(v0);
            GL.Vertex(v1);
            GL.Vertex(v2);
        }
        GL.End();
    }

    void simulateCloud()
    {
        simCloudMass = Mathf.Clamp(simCloudMass, 0.01f, 100.0f);

        //Keep cloud on screen with 'stretchy tether'

        float locWindForce = currWindForce;
        if ((currWindForce < 0.0f && cloudVelocity.x > 0.0f) || (currWindForce > 0.0f && cloudVelocity.x < 0.0f))
        {
            locWindForce *= 3.0f;
        }

        cloudVelocity.x += (locWindForce / simCloudMass) * Time.deltaTime;
        cloudPos.x += cloudVelocity.x * Time.deltaTime;
        if (cloudPos.x < 0.05f)
        {
            cloudVelocity.x = 0.0f;
            cloudPos.x = 0.05f;
        }
        else if (cloudPos.x > 0.8f)
        {
            cloudVelocity.x = 0.0f;
            cloudPos.x = 0.8f;
        }


        float vX = cloudPos.x;
        float vY = cloudPos.y;
        cloudVerts.Clear();
        cloudVerts.Add(new Vector3(vX, vY + 0.0f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.005f, vY + 0.0125f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.025f, vY + 0.02f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.03f, vY + 0.03f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.05f, vY + 0.05f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.07f, vY + 0.06f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.09f, vY + 0.065f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.11f, vY + 0.06f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.12f, vY + 0.0575f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.125f, vY + 0.045f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.145f, vY + 0.025f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.155f, vY + 0.005f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.175f, vY + 0.0075f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.18f, vY - 0.0025f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.15f, vY - 0.0125f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.1f, vY - 0.005f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.075f, vY - 0.0075f, 0.0f));
        cloudVerts.Add(new Vector3(vX + 0.045f, vY - 0.0085f, 0.0f));
        cloudVerts.Add(new Vector3(vX, vY + 0.0f, 0.0f));
    }
    
    void simulateTurkeys()
    {
        foreach (VerletTurkey turkey in turkeys )
        {
            turkey.move( Time.deltaTime );
            intersectTurkeyWithScene(turkey);
            //10% chance to jump every 3 seconds if not in the air;
            if (Time.time - turkey.lastJumpTime >= 1.5f)
            {
                turkey.lastJumpTime = Time.time;
                if (Random.Range(0, 9) == 0)
                {
                    turkey.jump( turkeyJumpSpeed );
                }
            }
        }
    }

    void drawTurkeys()
    {
        foreach (VerletTurkey turkey in turkeys)
        {
            turkey.draw();
        }
    }

    void intersectTurkeyWithScene( VerletTurkey turkey )
    {
        foreach (VerletPoint p in turkey.points)
        {
            if (p.pos.y <= 0)
            {
                turkey.displace(new Vector3(0, -p.pos.y, 0));
                if( turkey.inAir ) turkey.ground();
            }
            if (p.pos.y >= mountainTopY)
            {
                //turkey.displace(new Vector3(currWindForce * Time.deltaTime * 0.01f, 0, 0));
                //p.pos.x += currWindForce * Time.deltaTime * 0.01f;
            }

            // Cannonballs are sufficiently large releative to line-segment length in turkeys that we don't need to check 
            // intersection with line segments, only points
            for (int i = 0; i < cannonballs.Count; i++ )
            {
                Cannonball c = cannonballs[i];
                if (pointInCircle(p.pos, c.centre, c.radius))
                {
                    //Prevent the turkeys from getting too fast
                    float velX = Mathf.Clamp(c.velocity.x / 250, - turkeySpeed*0.5f, turkeySpeed*0.5f ) ;
                    turkey.addExternalVelocity(new Vector3( velX, 0.0f ));
                    cannonballs.RemoveAt(i);
                    i--;
                    continue;
                }
            }
        }

        //foreach (VerletConstraint c in turkey.constraints) c.resolve();
    }

    // True if cannonball should be removed
    bool intersectCannonballWithScene( ref Cannonball b )
    {
        // Check for intersection with ground 
        if (b.centre.y - b.radius <= 0.0f)
        {
            b.centre.y = b.radius;
            return true;
        }

        // Check if leaves screen right
        if (b.centre.y - b.radius >= 1.0f)
        {
            return true;
        }

        // Check if leaves screen top
        if (b.centre.x - b.radius >= 1.0f)
        {
            return true;
        }


        //Check for intersection with Wall
        // Uses static collision detection
        if (b.centre.x - b.radius <= wallStart+wallWidth)
        {
            b.centre.x = wallStart + wallWidth + b.radius;
            b.velocity.x = -b.velocity.x * ballBounceRestitution;
        }

        //Check for intersection with Mountain
        // Uses swept volume of cannonball
        Cannonball next = cannonballNextFrame(b);

        Rectangle sweptArea = new Rectangle();
        Vector3 bToNext = next.centre - b.centre;
        Vector3 bToNextNorm = new Vector3(-bToNext.y, bToNext.x);
        bToNextNorm.Normalize();

        sweptArea.a = b.centre + bToNextNorm * b.radius;
        sweptArea.b = b.centre - bToNextNorm * b.radius;
        sweptArea.c = next.centre + bToNextNorm * b.radius;
        sweptArea.d = next.centre - bToNextNorm * b.radius;

        float minIntDist = float.MaxValue;
        Cannonball res = new Cannonball(b);
        for (int i = 1; i < terrain.Count; i++)
        {
            Vector3 p0 = terrain[i-1];
            Vector3 p1 = terrain[i];
            Vector3? pInt;

            b.isGrounded = false;

            if (intersectLineSegWithCannonball(p0, p1, b, out pInt))
            {
                //b.centre = (Vector3)pInt;
                Cannonball temp = bounceOnIntersection(p0, p1, (Vector3)pInt, b);
                float intersectDist = (temp.centre - b.centre).magnitude;
                if (intersectDist < minIntDist)
                {
                    res = temp;
                    minIntDist = intersectDist;
                }
            }
            else if (intersectLineSegWithCannonball(p0, p1, next, out pInt))
            {
                //b.centre = (Vector3)pInt;
                Cannonball temp = bounceOnIntersection(p0, p1, (Vector3)pInt, b);
                float intersectDist = (temp.centre - b.centre).magnitude;
                if (intersectDist < minIntDist)
                {
                    res = temp;
                    minIntDist = intersectDist;
                }
            } 
            else if (intersectLineSegWithRectangle(p0, p1, sweptArea, out pInt))
            {
                //b.centre = (Vector3)pInt;
                Cannonball temp = bounceOnIntersection(p0, p1, (Vector3)pInt, b);
                float intersectDist = (temp.centre - b.centre).magnitude;
                if (intersectDist < minIntDist)
                {
                    res = temp;
                    minIntDist = intersectDist;
                }
            }
        }
        b.velocity      = res.velocity;
        b.centre        = res.centre;
        b.isGrounded    = res.isGrounded;


        return false;
    }

    Cannonball bounceOnIntersection(Vector3 p0, Vector3 p1, Vector3 pInt, Cannonball b)
    {
        Cannonball res = new Cannonball(b);
        Vector3 p01 = p1 - p0;
        Vector3 pBInt = pInt - b.centre;
        Vector3 n = new Vector3(-p01.y, p01.x);
        n.Normalize();
        n = (Vector3.Dot(pBInt, n) < 0) ? n : -n;
        Vector3 bounceDir = pBInt - 2 * Vector3.Dot(pBInt, -n) * -n;
        bounceDir.Normalize();
        res.velocity = b.velocity.magnitude * ballBounceRestitution * bounceDir;
        res.centre = pInt + b.radius * n * 1.01f;
        if (Mathf.Abs(b.velocity.y) < 0.05f)
        {
            res.velocity.y = 0;
            res.isGrounded = true;
        }
        return res;
    }

    bool intersectLineSegWithCannonballAlt(Vector3 s0, Vector3 s1, Cannonball b, out Vector3? pInt)
    {
        pInt = null;
        return false;
    }

    bool intersectLineSegWithCannonball( Vector3 s0, Vector3 s1, Cannonball b, out Vector3? pInt )
    {
        pInt = null;

        // Intersection happens if either point defining line is in circle
        bool s0inB = pointInCircle(s0, b.centre, b.radius);
        bool s1inB = pointInCircle(s1, b.centre, b.radius);
        if (s0inB && s1inB)
        {
            pInt = (s0 + s1) * 0.5f;
            return true;
        }
        else if (s0inB)
        {
            pInt = s0;
            return true;
        }
        else if (s1inB)
        {
            pInt = s1;
            return true;
        }


        Vector3 v01 = s1 - s0;
        Vector3 v0b = b.centre - s0;
        float d01_0b = Vector3.Dot(v01, v0b);
        Vector3 v0Int = (d01_0b / v01.magnitude) * v01;

        Vector3 tInt = v0Int + s0;
        float loX = Mathf.Min(s0.x, s1.x);
        float hiX = Mathf.Max(s0.x, s1.x);
        float loY = Mathf.Min(s0.y, s1.y);
        float hiY = Mathf.Max(s0.y, s1.y);
        if (tInt.x >= loX && tInt.x <= hiX && tInt.y >= loY && tInt.y <= hiY)
        {
            Vector3 vIntB = b.centre - tInt;
            if (vIntB.magnitude <= b.radius)
            {
                pInt = tInt;
                return true;
            }
        }
        return false;
    }

    bool pointInCircle(Vector3 p0, Vector3 c, float radius)
    {
        Vector3 v0c = c - p0;
        return v0c.magnitude <= radius;
    }

    bool intersectLineSegWithRectangle( Vector3 s0, Vector3 s1, Rectangle r, out Vector3? pInt )
    {
        pInt = null;
        pInt = intersectLineSegs(s0, s1, r.a, r.b);
        if (pInt == null) pInt = intersectLineSegs(s0, s1, r.b, r.c);
        if (pInt == null) pInt = intersectLineSegs(s0, s1, r.c, r.d);
        if (pInt == null) pInt = intersectLineSegs(s0, s1, r.d, r.a);
        return (pInt!=null);
    }

    Vector3? intersectLineSegs(Vector3 p00, Vector3 p01, Vector3 p10, Vector3 p11)
    {
        // calculate the distance to intersection point
        float uA = ((p11.x - p10.x) * (p00.y - p10.y) - (p11.y - p10.y) * (p00.x - p10.x)) / ((p11.y - p10.y) * (p01.x - p00.x) - (p11.x - p10.x) * (p01.y - p00.y));
        float uB = ((p01.x - p00.x) * (p00.y - p10.y) - (p01.y - p00.y) * (p00.x - p10.x)) / ((p11.y - p10.y) * (p01.x - p00.x) - (p11.x - p10.x) * (p01.y - p00.y));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1)
        {
            float intersectionX = p00.x + (uA * (p01.x - p00.x));
            float intersectionY = p00.y + (uA * (p01.y - p00.y));

            return new Vector3(intersectionX,intersectionY);
        }
        return null;
    }
}
