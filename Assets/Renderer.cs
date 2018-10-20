using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Renderer : MonoBehaviour
{
    // Draws a Triangle, a Quad and a line
    // with different colors

    List<Vector3> terrain = new List<Vector3>();

    public float wallStart = 0.1f;
    public float terrainStart = 0.125f;
    public float groundY = 0.15f;
    public float terrainEnd = 0.9f;


    void OnPostRender()
    {
        GL.PushMatrix();
        GL.LoadOrtho();

        GL.Clear( true, true, Color.white );

        GL.Begin( GL.QUADS ); // Quad
            GL.Color( new Color(0.0f,0.0f,0.0f) );
            GL.Vertex3( wallStart, 0.85f, 0 );
            GL.Vertex3( wallStart, groundY, 0 );
            GL.Vertex3( terrainStart, groundY, 0 );
            GL.Vertex3( terrainStart, 0.85f, 0 );
        GL.End();

        //generateTerrain(1, 0.1f);
        //drawTerrain();


        GL.PopMatrix();
    }

    void generateTerrain( int granularity, float offCoeff )
    {
        if ( terrain.Count <= 0 )
        {
            terrain.Add( new Vector3( terrainStart, groundY, 0.0f ) );
            terrain.Add( new Vector3( terrainEnd, groundY, 0.0f ) );
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
                newTerrain[2 * i] = terrain[i];
                newTerrain[2 * i + 1] = mid + Vector3.up * off;
            }
            newTerrain[newTerrain.Count - 1] = terrain[terrain.Count - 1];
            terrain = newTerrain;
        }
    }

    void drawTerrain()
    {
        GL.Begin( GL.LINES );
            for ( int i = 0; i < terrain.Count; i++ )
            {
                GL.Vertex( terrain[i] );
            }
        GL.End();
    }
}
