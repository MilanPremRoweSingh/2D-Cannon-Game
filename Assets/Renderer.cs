using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Renderer : MonoBehaviour
{
    // Draws a Triangle, a Quad and a line
    // with different colors

    List<Vector3> terrain = new List<Vector3>();


    public float wallStart;
    public float terrainStart;
    public float wallHeight;
    public float terrainEnd;
    public float spikiness;
    public int detailLevel;


    public float cannonRadius;
    public float cannonWidth;
    public float cannonLength;
    public float cannonXPos;
    public float cannonNumVerts;
    private float cannonAngle;


    private void Start()
    {
        generateTerrain(detailLevel, spikiness);
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
            GL.Vertex3( 0.025f, 0.0f, 0.0f );
            GL.Vertex3( 0.025f, wallHeight, 0.0f );
        GL.End();

        drawTerrain();

        drawCircle(cannonRadius, new Vector3(cannonXPos, cannonRadius, 0.0f));

        GL.PopMatrix();
    }

    void generateTerrain( int granularity, float offCoeff )
    {
        if (granularity == 0)
        {
            return;
        }

        // Genearate Initial Mountain
        if ( terrain.Count <= 0 )
        {
            terrain.Add( new Vector3( terrainStart, 0.0f, 0.0f ) );

            float diff = terrainEnd - terrainStart;
            terrain.Add(new Vector3(terrainStart + diff*0.375f, 0.5f, 0.0f));
            terrain.Add(new Vector3(terrainStart + diff*0.625f, 0.5f, 0.0f));

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
                newTerrain.Add(mid + Vector3.up * off);
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


}
