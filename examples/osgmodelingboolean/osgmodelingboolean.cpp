/* -*-c++-*- osgModeling Example: Boolean operating tools
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
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgUtil/TriStripVisitor>
#include <osgViewer/Viewer>

#include <osgModeling/Extrude>
#include <osgModeling/Lathe>
#include <osgModeling/Nurbs>
#include <osgModeling/BoolOperator>

osg::ref_ptr<osg::Geometry> createFirstOperator()
{
#if 0
    osg::ref_ptr<osgModeling::Curve> profile = new osgModeling::Curve;
    profile->addPathPoint( osg::Vec3(-0.2f, 0.0f, 0.0f) );
    profile->addPathPoint( osg::Vec3(-0.2f, 0.0f, 2.0f) );
    osg::ref_ptr<osgModeling::Lathe> geom = new osgModeling::Lathe;
    geom->setGenerateParts( osgModeling::Model::ALL_PARTS );
    geom->setLatheSegments( 16 );
    geom->setLatheOrigin( osg::Vec3(0.0f, 0.0f, -0.8f) );
    geom->setLatheAxis( osg::Vec3(0.0f, 0.0f, 1.0f) );
    geom->setProfile( profile.get() );
    geom->update();
#else
    double cp[5][3] = {
        {1.0f,0.0f,1.0f}, {-1.0f,0.0f,1.0f}, {-1.0f,0.0f,-1.0f},
        {1.0f,0.0f,-1.0f}, {1.0f,0.0f,1.0f} };
    osg::ref_ptr<osgModeling::Curve> profile = new osgModeling::Curve;
    profile->setPath( 15, &cp[0][0] );
    osg::ref_ptr<osgModeling::Extrude> geom = new osgModeling::Extrude;
    geom->setGenerateParts( osgModeling::Model::ALL_PARTS );
    geom->setExtrudeDirection( osg::Vec3(0.0f, 1.0f, 0.0f) );
    geom->setExtrudeLength( 1.0f );
    geom->setProfile( profile.get() );
    geom->update();
#endif
    return geom.get();
}

osg::ref_ptr<osg::Geometry> createSecondOperator()
{
#if 1
    double r = 0.5f;
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
    osg::ref_ptr<osgModeling::NurbsSurface> geom = new osgModeling::NurbsSurface(
        12, &knotsU[0], 8, &knotsV[0], 20, 4, &ctrlAndWeightPts[0][0][0], 3, 3, 16, 16 );
#else
    double cp[5][3] = {
        {0.5f,1.0f,0.0f}, {-0.5f,0.0f,0.0f}, {0.5f,-1.0f,0.0f},
        {1.5f,0.0f,0.0f}, {0.5f,1.0f,0.0f} };
    osg::ref_ptr<osgModeling::Curve> profile = new osgModeling::Curve;
    profile->setPath( 15, &cp[0][0] );
    osg::ref_ptr<osgModeling::Extrude> geom = new osgModeling::Extrude;
    geom->setGenerateParts( osgModeling::Model::ALL_PARTS );
    geom->setExtrudeLength( 2.0f );
    geom->setProfile( profile.get() );
    geom->update();
#endif
    return geom.get();
}

osg::ref_ptr<osg::Node> createBoolean()
{
    // A boolean operation requires input Model instances because it uses the BSP tree to finish the work.
    // We could generate Model objects both from Geometry instances or derived classes (e.g. Extrude, Lathe).
    osg::ref_ptr<osgModeling::Model> model1 = new osgModeling::Model( *createFirstOperator() );
    osg::ref_ptr<osgModeling::Model> model2 = new osgModeling::Model( *createSecondOperator() );

    // Choose boolean method: Intersection, Union or Difference.
    osg::ref_ptr<osgModeling::BoolOperator> boolOp = new osgModeling::BoolOperator;
    boolOp->setMethod( osgModeling::BoolOperator::BOOL_DIFFERENCE );
    boolOp->setOperands( model1.get(), model2.get() );

    // Calculate and output the result into a new geometry.
    // Be careful, it may cost long time or even crash if you input a too complex model.
    osg::ref_ptr<osg::Geometry> result = new osg::Geometry;
    boolOp->output( result.get() );
    
    // A triangle strip generator should be used here, otherwise too many independent triangles may cause the graphics system crash. 
    osgUtil::TriStripVisitor tsv;
    tsv.stripify( *result );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( result.get() );
    return geode;
}

int main( int argc, char** argv )
{
    // The result of boolean operations may NOT be good looking at present. In fact, it's always full of disordered meshes
    // and distorted normals. That's because I haven't done any optimizations to the BSP nodes, the face partition algorithm,
    // and the generated geometry. There must be a long way to go...
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( createBoolean().get() );

    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    viewer.run();

    // You could save the geometry data to a file and review/modify it later. And this might be the main reason to have a
    // computational boolean method here.
    // If just for displaying, maybe you would choose the CSG method using stencil buffers instead. I will consider to implement
    // the real-time CSG boolean operator in future versions.
    osgDB::writeNodeFile( *root, "./node.osg" );
    return 0;
}
