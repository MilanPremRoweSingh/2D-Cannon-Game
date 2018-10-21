using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cannonball
{
    public const float mass = 1;
    public Vector3 centre;
    public Vector3 velocity;
    public float radius;

    public Cannonball( Vector3 _centre, Vector3 _velocity, float _radius )
    {
        centre   = _centre;
        velocity = _velocity;
        radius   = _radius;
    }

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
            terrain.Add(new Vector3(terrainStart + diff*0.375f, 0.5f, 0.0f));
            terrain.Add(new Vector3(terrainStart + diff*0.625f, 0.5f, 0.0f));
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
            curr.velocity.y -= gravity * Time.deltaTime;
            if (curr.centre.y >= mountainTopY)
            {
                curr.velocity.x += (currWindForce / Cannonball.mass) * Time.deltaTime;
            }
            curr.centre += curr.velocity * Time.deltaTime;

            intersectCannonballWithScene(curr);
        }
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

    // Uses static collision
    void intersectCannonballWithScene( Cannonball b )
    {
        //Check for intersection with Wall
        if (b.centre.x - (b.radius / 2.0f) <= wallStart+wallWidth)
        {
            b.centre.x = wallStart + wallWidth + (b.radius / 2.0f);
            b.velocity.x = -b.velocity.x * ballBounceRestitution;
        }

        //Check for intersection with Mountain
    }
}
