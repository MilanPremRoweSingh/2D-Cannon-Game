using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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


public class Renderer : MonoBehaviour
{
    // Draws a Triangle, a Quad and a line
    // with different colors

    List<Vector3> terrain = new List<Vector3>();
    List<Cannonball> cannonballs = new List<Cannonball>();
    List<Vector3> cloudVerts = new List<Vector3>();


    public float wallStart;
    public float wallWidth;
    public float terrainStart;
    public float wallHeight;
    public float terrainEnd;
    public float spikiness;
    public int detailLevel;


    public float cannonRadius;
    public float cannonWidth;
    public float cannonLength;
    public float cannonXPos;
    public float cannonSpeed;
    private float cannonAngle = -Mathf.PI/4;

    public float ballInitialSpeed;
    public float ballMinSpeed;
    [Range(0.5f,0.95f)]
    public float ballBounceRestitution;

    public float gravity = 9.81f;
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
    }

    private void Update()
    {
        controlCannon();
        simulateCannonballs();
        simulateWind();
        simulateCloud();
    }

    void OnPostRender()
    {
        GL.PushMatrix();
        GL.LoadOrtho();

        GL.Clear( true, true, Color.white );

        GL.Begin( GL.QUADS ); // Quad
            GL.Color( new Color(0.25f,0.25f,0.25f) );
            GL.Vertex3( 0.0f, wallHeight, 0.0f );
            GL.Vertex3( 0.0f, 0.0f, 0.0f );
            GL.Vertex3( wallWidth, 0.0f, 0.0f );
            GL.Vertex3( wallWidth, wallHeight, 0.0f );
        GL.End();

        drawTerrain();
        drawCloud();
        drawCannon();
        drawCannonballs();


        GL.PopMatrix();
    }

    void generateTerrain( int granularity, float offCoeff )
    {
        if (granularity == 0)
        {
            return;
        }
        mountainTopY = 0.0f;
        // Genearate Initial Mountain
        if ( terrain.Count <= 0 )
        {
            terrain.Add( new Vector3( terrainStart, 0.0f, 0.0f ) );

            float diff = terrainEnd - terrainStart;
            terrain.Add(new Vector3(terrainStart + diff*0.375f, 0.45f, 0.0f));
            terrain.Add(new Vector3(terrainStart + diff*0.625f, 0.45f, 0.0f));
            mountainTopY = 0.5f;

            terrain.Add( new Vector3( terrainEnd, 0.0f, 0.0f ) );
        }
        else
        {
            List<Vector3> newTerrain = new List<Vector3>( terrain.Count*2 - 1 );
            for ( int i = 0; i < terrain.Count - 1; i++ )
            {
                Vector3 p0 = terrain[i];
                Vector3 p1 = terrain[i+1];
                float len = ( p0 - p1 ).magnitude;

                Vector3 mid = ( p0 + p1 ) * 0.5f;
                float off = offCoeff * Random.Range( -len, len );
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
        GL.Begin( GL.LINE_STRIP );
            for ( int i = 0; i < terrain.Count; i++ )
            {
                GL.Vertex( terrain[i] );
            }
        GL.End();
    }

    void drawCircle( float radius, Vector3 centre )
    {
        GL.Begin(GL.TRIANGLES);
        float delta = 0.01f;
        float theta = 0.0f;
        while( theta <= 2*Mathf.PI )
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

        Vector3 cannonPos = new Vector3( cannonXPos, (2*cannonRadius-cannonWidth/2) );
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
            Vector3 velocity  = aimVector.normalized;
            velocity *= ballInitialSpeed;
            Vector3 startPos = new Vector3(cannonXPos, (2 * cannonRadius - cannonWidth / 2));
            startPos += aimVector * cannonLength;
            cannonballs.Add(new Cannonball( startPos, velocity, 1.05f*cannonWidth/2.0f ));
        }
    }

    void simulateCannonballs()
    {
       
        for (int i = 0; i < cannonballs.Count; i++)
        {
            Cannonball curr = cannonballs[i];
            if (intersectCannonballWithScene( ref curr) )
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
        if (currTime -timeLastWindchange >= 0.5f)
        {
            currWindForce = Random.Range(-maxWindMag, maxWindMag);
            timeLastWindchange = currTime;
        }
    }

    void drawCloud()
    {
        GL.Begin(GL.TRIANGLES);
        GL.Color(Color.white);
        for (int i = 0; i < cloudVerts.Count-1; i++)
        {
            Vector3 v0 = cloudVerts[i];
            Vector3 v1 = cloudPos;
            Vector3 v2 = cloudVerts[i+1];

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
        if ( (currWindForce < 0.0f && cloudVelocity.x > 0.0f) || (currWindForce > 0.0f && cloudVelocity.x < 0.0f) )
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
