/* -*-c++-*- osgModeling Example: Bezier and NURBS surface tools
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.

* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.

* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <osg/Geode>
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osgModeling/Loft>
#include <osgModeling/Bezier>
#include <osgModeling/Nurbs>

// The classic OpenGL teapot... taken form glut-3.7/lib/glut/glut_teapot.c

/* Copyright (c) Mark J. Kilgard, 1994. */

/**
(c) Copyright 1993, Silicon Graphics, Inc.

ALL RIGHTS RESERVED

Permission to use, copy, modify, and distribute this software
for any purpose and without fee is hereby granted, provided
that the above copyright notice appear in all copies and that
both the copyright notice and this permission notice appear in
supporting documentation, and that the name of Silicon
Graphics, Inc. not be used in advertising or publicity
pertaining to distribution of the software without specific,
written prior permission.

THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU
"AS-IS" AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR
OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  IN NO
EVENT SHALL SILICON GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE
ELSE FOR ANY DIRECT, SPECIAL, INCIDENTAL, INDIRECT OR
CONSEQUENTIAL DAMAGES OF ANY KIND, OR ANY DAMAGES WHATSOEVER,
INCLUDING WITHOUT LIMITATION, LOSS OF PROFIT, LOSS OF USE,
SAVINGS OR REVENUE, OR THE CLAIMS OF THIRD PARTIES, WHETHER OR
NOT SILICON GRAPHICS, INC.  HAS BEEN ADVISED OF THE POSSIBILITY
OF SUCH LOSS, HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
ARISING OUT OF OR IN CONNECTION WITH THE POSSESSION, USE OR
PERFORMANCE OF THIS SOFTWARE.

US Government Users Restricted Rights

Use, duplication, or disclosure by the Government is subject to
restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
(c)(1)(ii) of the Rights in Technical Data and Computer
Software clause at DFARS 252.227-7013 and/or in similar or
successor clauses in the FAR or the DOD or NASA FAR
Supplement.  Unpublished-- rights reserved under the copyright
laws of the United States.  Contractor/manufacturer is Silicon
Graphics, Inc., 2011 N.  Shoreline Blvd., Mountain View, CA
94039-7311.

OpenGL(TM) is a trademark of Silicon Graphics, Inc.
*/

/* Rim, body, lid, and bottom data must be reflected in x and
y; handle and spout data across the y axis only.  */

static int patchdata[][16] =
{
    /* rim */
    {102, 103, 104, 105, 4, 5, 6, 7, 8, 9, 10, 11,
    12, 13, 14, 15},
    /* body */
    {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27},
    {24, 25, 26, 27, 29, 30, 31, 32, 33, 34, 35, 36,
    37, 38, 39, 40},
    /* lid */
    {96, 96, 96, 96, 97, 98, 99, 100, 101, 101, 101,
    101, 0, 1, 2, 3,},
    {0, 1, 2, 3, 106, 107, 108, 109, 110, 111, 112,
    113, 114, 115, 116, 117},
    /* bottom */
    {118, 118, 118, 118, 124, 122, 119, 121, 123, 126,
    125, 120, 40, 39, 38, 37},
    /* handle */
    {41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52,
    53, 54, 55, 56},
    {53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64,
    28, 65, 66, 67},
    /* spout */
    {68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83},
    {80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91,
    92, 93, 94, 95}
};

static float cpdata[][3] =
{
    {0.2, 0, 2.7}, {0.2, -0.112, 2.7}, {0.112, -0.2, 2.7}, {0,
    -0.2, 2.7}, {1.3375, 0, 2.53125}, {1.3375, -0.749, 2.53125},
    {0.749, -1.3375, 2.53125}, {0, -1.3375, 2.53125}, {1.4375,
    0, 2.53125}, {1.4375, -0.805, 2.53125}, {0.805, -1.4375,
    2.53125}, {0, -1.4375, 2.53125}, {1.5, 0, 2.4}, {1.5, -0.84,
    2.4}, {0.84, -1.5, 2.4}, {0, -1.5, 2.4}, {1.75, 0, 1.875},
    {1.75, -0.98, 1.875}, {0.98, -1.75, 1.875}, {0, -1.75,
    1.875}, {2, 0, 1.35}, {2, -1.12, 1.35}, {1.12, -2, 1.35},
    {0, -2, 1.35}, {2, 0, 0.9}, {2, -1.12, 0.9}, {1.12, -2,
    0.9}, {0, -2, 0.9}, {-2, 0, 0.9}, {2, 0, 0.45}, {2, -1.12,
    0.45}, {1.12, -2, 0.45}, {0, -2, 0.45}, {1.5, 0, 0.225},
    {1.5, -0.84, 0.225}, {0.84, -1.5, 0.225}, {0, -1.5, 0.225},
    {1.5, 0, 0.15}, {1.5, -0.84, 0.15}, {0.84, -1.5, 0.15}, {0,
    -1.5, 0.15}, {-1.6, 0, 2.025}, {-1.6, -0.3, 2.025}, {-1.5,
    -0.3, 2.25}, {-1.5, 0, 2.25}, {-2.3, 0, 2.025}, {-2.3, -0.3,
    2.025}, {-2.5, -0.3, 2.25}, {-2.5, 0, 2.25}, {-2.7, 0,
    2.025}, {-2.7, -0.3, 2.025}, {-3, -0.3, 2.25}, {-3, 0,
    2.25}, {-2.7, 0, 1.8}, {-2.7, -0.3, 1.8}, {-3, -0.3, 1.8},
    {-3, 0, 1.8}, {-2.7, 0, 1.575}, {-2.7, -0.3, 1.575}, {-3,
    -0.3, 1.35}, {-3, 0, 1.35}, {-2.5, 0, 1.125}, {-2.5, -0.3,
    1.125}, {-2.65, -0.3, 0.9375}, {-2.65, 0, 0.9375}, {-2,
    -0.3, 0.9}, {-1.9, -0.3, 0.6}, {-1.9, 0, 0.6}, {1.7, 0,
    1.425}, {1.7, -0.66, 1.425}, {1.7, -0.66, 0.6}, {1.7, 0,
    0.6}, {2.6, 0, 1.425}, {2.6, -0.66, 1.425}, {3.1, -0.66,
    0.825}, {3.1, 0, 0.825}, {2.3, 0, 2.1}, {2.3, -0.25, 2.1},
    {2.4, -0.25, 2.025}, {2.4, 0, 2.025}, {2.7, 0, 2.4}, {2.7,
    -0.25, 2.4}, {3.3, -0.25, 2.4}, {3.3, 0, 2.4}, {2.8, 0,
    2.475}, {2.8, -0.25, 2.475}, {3.525, -0.25, 2.49375},
    {3.525, 0, 2.49375}, {2.9, 0, 2.475}, {2.9, -0.15, 2.475},
    {3.45, -0.15, 2.5125}, {3.45, 0, 2.5125}, {2.8, 0, 2.4},
    {2.8, -0.15, 2.4}, {3.2, -0.15, 2.4}, {3.2, 0, 2.4}, {0, 0,
    3.15}, {0.8, 0, 3.15}, {0.8, -0.45, 3.15}, {0.45, -0.8,
    3.15}, {0, -0.8, 3.15}, {0, 0, 2.85}, {1.4, 0, 2.4}, {1.4,
    -0.784, 2.4}, {0.784, -1.4, 2.4}, {0, -1.4, 2.4}, {0.4, 0,
    2.55}, {0.4, -0.224, 2.55}, {0.224, -0.4, 2.55}, {0, -0.4,
    2.55}, {1.3, 0, 2.55}, {1.3, -0.728, 2.55}, {0.728, -1.3,
    2.55}, {0, -1.3, 2.55}, {1.3, 0, 2.4}, {1.3, -0.728, 2.4},
    {0.728, -1.3, 2.4}, {0, -1.3, 2.4}, {0, 0, 0}, {1.425,
    -0.798, 0}, {1.5, 0, 0.075}, {1.425, 0, 0}, {0.798, -1.425,
    0}, {0, -1.5, 0.075}, {0, -1.425, 0}, {1.5, -0.84, 0.075},
    {0.84, -1.5, 0.075}
};

osg::ref_ptr<osg::Geode> createBezierTeapot()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    double p[4][4][3], q[4][4][3], r[4][4][3], s[4][4][3];
    long i, j, k, l;
    for ( i = 0; i < 10; i++ )
    {
        for ( j = 0; j < 4; j++ )
        {
            for ( k = 0; k < 4; k++ )
            {
                for ( l = 0; l < 3; l++ )
                {
                    p[j][k][l] = cpdata[patchdata[i][j * 4 + k]][l];
                    q[j][k][l] = cpdata[patchdata[i][j * 4 + (3 - k)]][l];
                    if ( l == 1 )
                        q[j][k][l] *= -1.0;
                    if ( i < 6 )
                    {
                        r[j][k][l] =
                            cpdata[patchdata[i][j * 4 + (3 - k)]][l];
                        if ( l == 0 )
                            r[j][k][l] *= -1.0;
                        s[j][k][l] = cpdata[patchdata[i][j * 4 + k]][l];
                        if ( l == 0 )
                            s[j][k][l] *= -1.0;
                        if ( l == 1 )
                            s[j][k][l] *= -1.0;
                    }
                }
            }
        }
        geode->addDrawable( new osgModeling::BezierSurface(3, 4, 12, 4, &p[0][0][0], 10, 10) );
        geode->addDrawable( new osgModeling::BezierSurface(3, 4, 12, 4, &q[0][0][0], 10, 10) );
        if ( i < 6 )
        {
            geode->addDrawable( new osgModeling::BezierSurface(3, 4, 12, 4, &r[0][0][0], 10, 10) );
            geode->addDrawable( new osgModeling::BezierSurface(3, 4, 12, 4, &s[0][0][0], 10, 10) );
        }
    }
    return geode;
}

osg::ref_ptr<osg::Geode> createNurbsCircle()
{
    // A standard NURBS circle has 9 control points forming a square.
    osg::ref_ptr<osg::Vec3Array> ctrlPts = new osg::Vec3Array;
    ctrlPts->push_back(osg::Vec3( 2.0f,0.0f, 0.0f)); ctrlPts->push_back(osg::Vec3( 2.0f, 2.0f, 0.0f));
    ctrlPts->push_back(osg::Vec3( 0.0f,2.0f, 0.0f)); ctrlPts->push_back(osg::Vec3(-2.0f, 2.0f, 0.0f));
    ctrlPts->push_back(osg::Vec3(-2.0f,0.0f, 0.0f)); ctrlPts->push_back(osg::Vec3(-2.0f,-2.0f, 0.0f));
    ctrlPts->push_back(osg::Vec3( 0.0f,-2.0f,0.0f)); ctrlPts->push_back(osg::Vec3( 2.0f,-2.0f, 0.0f));
    ctrlPts->push_back(osg::Vec3( 2.0f,0.0f, 0.0f));

    // The weights of 4 corners are less than 1.0, so they won't 'drag' the curve too much.
    osg::ref_ptr<osg::DoubleArray> weightPts = new osg::DoubleArray;
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f ); weightPts->push_back( sqrt(2.f)/2.0f );
    weightPts->push_back( 1.0f );

    // Set the knot vector. Vector size equals to number of control points + order(degree+1) of curve.
    osg::ref_ptr<osg::DoubleArray> knotPts = new osg::DoubleArray;
    knotPts->push_back( 0.0f ); knotPts->push_back( 0.0f );
    knotPts->push_back( 0.0f ); knotPts->push_back( 1.0f );
    knotPts->push_back( 1.0f ); knotPts->push_back( 2.0f );
    knotPts->push_back( 2.0f ); knotPts->push_back( 3.0f );
    knotPts->push_back( 3.0f ); knotPts->push_back( 4.0f );
    knotPts->push_back( 4.0f ); knotPts->push_back( 4.0f );

    // The code below shows 2 methods to configure the NURBS curve.
    // We may also use a third constructor, which is like the gluNurbsCurve, to easily read OpenGL curve data.
#if 0
    osg::ref_ptr<osgModeling::NurbsCurve> nurbsCurve = new osgModeling::NurbsCurve;
    nurbsCurve->setDegree( 2 );
    nurbsCurve->setNumPath( 100 );
    nurbsCurve->setKnotVector( knotPts.get() );
    nurbsCurve->setWeights( weightPts.get() );
    nurbsCurve->setCtrlPoints( ctrlPts.get() );
    nurbsCurve->update();
#else
    osg::ref_ptr<osgModeling::NurbsCurve> nurbsCurve =
        new osgModeling::NurbsCurve( ctrlPts.get(), weightPts.get(), knotPts.get(), 2, 100 );
#endif

    // use the loft tool to show the curve generated.
    osg::ref_ptr<osgModeling::Curve> section = new osgModeling::Curve;
    section->addPathPoint( osg::Vec3(-0.1f,0.0f,0.0f) );
    section->addPathPoint( osg::Vec3(0.1f,0.0f,0.0f) );
    osg::ref_ptr<osgModeling::Loft> circleLoft =
        new osgModeling::Loft( nurbsCurve.get(), section.get() );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( circleLoft.get() );
    return geode;
}

osg::ref_ptr<osg::Geode> createNurbsSphere()
{
    double r = 2.0f;
    double knotsU[12]= { 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4 };
    double knotsV[8] = { 0, 0, 0, 1, 1, 2, 2, 2 };
    double ctrlAndWeightPts[9][5][4] = {
        {{0,0,r,1}, { r, 0,r,1}, { r, 0,0,2}, { r, 0,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, { r,-r,r,1}, { r,-r,0,2}, { r,-r,-r,1}, {0,0,-r,1}},
        {{0,0,r,2}, { 0,-r,r,2}, { 0,-r,0,4}, { 0,-r,-r,2}, {0,0,-r,2}},
        {{0,0,r,1}, {-r,-r,r,1}, {-r,-r,0,2}, {-r,-r,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, {-r, 0,r,1}, {-r, 0,0,2}, {-r, 0,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, {-r, r,r,1}, {-r, r,0,2}, {-r, 1,-r,1}, {0,0,-r,1}},
        {{0,0,r,2}, { 0, r,r,2}, { 0, r,0,4}, { 0, r,-r,2}, {0,0,-r,2}},
        {{0,0,r,1}, { r, r,r,1}, { r, r,0,2}, { r, r,-r,1}, {0,0,-r,1}},
        {{0,0,r,1}, { r, 0,r,1}, { r, 0,0,2}, { r, 0,-r,1}, {0,0,-r,1}} };

    // Create a NURBS sphere with 16x16 segments. We could use gluNurbsSurface to finish the same thing,
    // but this class just converts it into osg::Geometry and makes it possible to generate NURBS objects
    // with different level of details (LOD) and save vertices information if required.
    osg::ref_ptr<osgModeling::NurbsSurface> sphere = new osgModeling::NurbsSurface(
        12, &knotsU[0], 8, &knotsV[0], 20, 4, &ctrlAndWeightPts[0][0][0], 3, 3, 16, 16 );

    // Surface points along the U direction will be given an average distribution on the S direction of texture,
    // and V corresponds to T. So we may have to flip our image to fit the auto-generated texcoords sometime.
    // It is suggested to use customized texcoords or osg::TexGen node instead.
    sphere->getOrCreateStateSet()->setTextureAttributeAndModes(
        0, new osg::Texture2D(osgDB::readImageFile("land_shallow_topo_2048_flip.jpg")) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( sphere.get() );
    return geode;
}

int main( int argc, char** argv )
{
    osg::ref_ptr<osg::PositionAttitudeTransform> teapot = new osg::PositionAttitudeTransform;
    teapot->setPosition( osg::Vec3(-8.0f, 0.0f, 0.0f) );
    teapot->addChild( createBezierTeapot().get() );

    osg::ref_ptr<osg::PositionAttitudeTransform> circle = new osg::PositionAttitudeTransform;
    circle->setPosition( osg::Vec3(0.0f, 0.0f, 0.0f) );
    circle->addChild( createNurbsCircle().get() );

    osg::ref_ptr<osg::PositionAttitudeTransform> sphere = new osg::PositionAttitudeTransform;
    sphere->setPosition( osg::Vec3(8.0f, 0.0f, 0.0f) );
    sphere->addChild( createNurbsSphere().get() );

    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( teapot.get() );
    root->addChild( circle.get() );
    root->addChild( sphere.get() );

    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    return viewer.run();
}
