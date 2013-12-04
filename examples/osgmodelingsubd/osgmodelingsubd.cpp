/* -*-c++-*- osgModeling Example: Subdivision tools
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
#include <osgUtil/TriStripVisitor>
#include <osgViewer/Viewer>

#include <osgModeling/Utilities>
#include <osgModeling/Subdivision>

osg::ref_ptr<osg::Geode> createSubd( osg::Drawable* drawable, int method, int level )
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    std::string methodName;
    osg::ref_ptr<osgModeling::Subdivision> subd;
    if ( !method )
    {
        methodName = "Loop";
        subd = new osgModeling::LoopSubdivision( level );
    }
    else
    {
        methodName = "Sqrt(3)";
        subd = new osgModeling::Sqrt3Subdivision( level );
    }

    std::cout << "*** Constructing the polygon mesh ..." << std::endl;
    osg::Timer_t t1 = osg::Timer::instance()->tick();

    osg::Geometry* geom = dynamic_cast<osg::Geometry*>( drawable );
    osg::ref_ptr<osgModeling::PolyMesh> mesh = new osgModeling::PolyMesh( *geom );
    geode->addDrawable( mesh.get() );

    osg::Timer_t t2 = osg::Timer::instance()->tick();
    std::cout << "- Edges: " << mesh->_edges.size() << std::endl;
    std::cout << "- Triangle Faces: " << mesh->_faces.size() << std::endl;
    std::cout << "- Constructing Time: " << osg::Timer::instance()->delta_s( t1, t2 ) << "s" << std::endl;

    if ( mesh->_faces.size()>2000 )
    {
        std::cout << "It needs a long time operating on thousands of faces. Maybe not necessary to subdivide such a fine model?" << std::endl;
        return geode;
    }

    std::cout << "*** Start subdividing using " << methodName << ", level " << level << " ... Please wait with patient." << std::endl;
    t1 = osg::Timer::instance()->tick();

    mesh->subdivide( subd );

    // A triangle strip generator should be used here, otherwise too many independent triangles may cause the graphics system crash.
    osgUtil::TriStripVisitor tsv;
    tsv.stripify( *mesh );

    t2 = osg::Timer::instance()->tick();
    std::cout << "- Subdividing Edges: " << mesh->_edges.size() << std::endl;
    std::cout << "- Subdividing Faces: " << mesh->_faces.size() << std::endl;
    std::cout << "- Subdividing Time Spend: " << osg::Timer::instance()->delta_s( t1, t2 ) << "s" << std::endl;

    return geode;
}

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName()+" is an example showing how to subdivide polygons." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName()+" [options] filename ..." );
    arguments.getApplicationUsage()->addCommandLineOption( "--method", "Set a subdivision algorithm, 'loop' and 'sqrt3' available at present." );
    arguments.getApplicationUsage()->addCommandLineOption( "--level", "Set level of the subdivision operation." );
    arguments.getApplicationUsage()->addCommandLineOption( "-h or --help","Display help documents." );

    if ( arguments.read("-h") || arguments.read("--help") )
    {
        std::cout << arguments.getApplicationUsage()->getCommandLineUsage() << std::endl;
        arguments.getApplicationUsage()->write( std::cout, arguments.getApplicationUsage()->getCommandLineOptions() );
        return 1;
    }

    int method, level;
    std::string methodName;
    if ( !arguments.read("--level", level) )
        level = 2;
    if ( !arguments.read("--method", methodName) )
        method = 1;
    else if ( methodName=="sqrt3" )
        method = 1;
    else
        method = 0;

    osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>( osgDB::readNodeFiles(arguments) );
    if ( !group.valid() || !group->getNumChildren() )
        group = dynamic_cast<osg::Group*>( osgDB::readNodeFile("./pawn.osg") );
    if ( !group.valid() || !group->getNumChildren() )
    {
        std::cout << "This example only accepts a root group node with one geode attached." << std::endl;
        return 1;
    }

    osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode*>( group->getChild(0) );
    if ( !geode.valid() || !geode->getNumDrawables() )
    {
        std::cout << "This example only accepts a root group node with one geode attached." << std::endl;
        return 1;
    }

    osgViewer::Viewer viewer;
    viewer.setSceneData( createSubd(geode->getDrawable(0), method, level).get() );
    return viewer.run();
}
