/* -*-c++-*- osgModeling Example: BSP tree for specified geometries
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

#include <string>
#include <iostream>
#include <osg/Geode>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>

#include <osgModeling/Model>
#include <osgModeling/ModelVisitor>


class ConvertToBspVisitor : public osg::NodeVisitor
{
public:
    ConvertToBspVisitor( unsigned int samples=5 ) : _samples(samples)
    { setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN); }
    virtual ~ConvertToBspVisitor() {}

    virtual void apply( osg::Geode& geode )
    {
        for ( unsigned int i=0; i<geode.getNumDrawables(); ++i )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry*>( geode.getDrawable(i) );
            if ( geom ) processBsp( *geom, i );
        }
    }

    void processBsp( osg::Geometry& geoset, unsigned int pos )
    {
        int maxDepth=0, numPosNode=0, numNegNode=0;
        std::cout << "===== BSP construction of geometry ";
        if ( geoset.getName().empty() ) std::cout << pos << " =====" << std::endl;
        else std::cout << ": " << geoset.getName().c_str() << " =====" << std::endl;

        osg::Vec3Array* vert = dynamic_cast<osg::Vec3Array*>( geoset.getVertexArray() );
        std::cout << "- Vertices Number: " << vert->size() << std::endl;

        osg::Timer_t t1 = osg::Timer::instance()->tick();

        osg::ref_ptr<osgModeling::Model> model = new osgModeling::Model( geoset );
        model->setBspTree( new osgModeling::BspTree(_samples) );
        osgModeling::ModelVisitor::buildBSP( *model );

        osg::Timer_t t2 = osg::Timer::instance()->tick();

        std::cout << "- Available Faces: " << model->getBspTree()->getFaceList().size() << std::endl;
        std::cout << "- Constructing Time: " << osg::Timer::instance()->delta_s( t1, t2 ) << "s" << std::endl;

        t1 = osg::Timer::instance()->tick();
        traverseBspNode( model->getBspTree()->getRoot(), 1, maxDepth, numPosNode, numNegNode );
        t2 = osg::Timer::instance()->tick();

        std::cout << "- Traversal Time: " << osg::Timer::instance()->delta_s( t1, t2 ) << "s" << std::endl;
        std::cout << "- Maximum Depth: " << maxDepth << std::endl;
        std::cout << "- Positive Nodes: " << numPosNode << std::endl;
        std::cout << "- Negative Nodes: " << numNegNode << std::endl;
    }

    void traverseBspNode( osgModeling::BspTree::BspNode* node, int depth,
                          int& maxDepth, int& posNodes, int& negNodes )
    {
        if ( !node ) return;
        if ( maxDepth<depth ) maxDepth = depth;

        if ( node->_posChild )
        {
            posNodes++;
            traverseBspNode( node->_posChild, depth+1, maxDepth, posNodes, negNodes );
        }
        if ( node->_negChild )
        {
            negNodes++;
            traverseBspNode( node->_negChild, depth+1, maxDepth, posNodes, negNodes );
        }
    }

protected:
    unsigned int _samples;
};

int main( int argc, char** argv )
{
    osg::ArgumentParser arguments( &argc, argv );
    arguments.getApplicationUsage()->setApplicationName( arguments.getApplicationName() );
    arguments.getApplicationUsage()->setDescription( arguments.getApplicationName()+" is an example showing how to build BSP trees for geodes." );
    arguments.getApplicationUsage()->setCommandLineUsage( arguments.getApplicationName()+" [options] filename ..." );
    arguments.getApplicationUsage()->addCommandLineOption( "-h or --help","Display help documents." );
    arguments.getApplicationUsage()->addCommandLineOption( "--samples",
        "Set a sampling number for dividing faces while constructing BSP trees. Default is 5, "
        "which means at most 5 faces of all will be checked to find best dividers in constructing of every node.\n"
        "The greater the number is, more faces will be sampled and the generated BSP tree MAY be more balanced (NOT ABSOLUTELY). "
        "The constructor will check all the faces if set to 0, but it will take a long long time to wait." );

    if ( arguments.read("-h") || arguments.read("--help") )
    {
        std::cout << arguments.getApplicationUsage()->getCommandLineUsage() << std::endl;
        arguments.getApplicationUsage()->write( std::cout, arguments.getApplicationUsage()->getCommandLineOptions() );
        return 1;
    }

    unsigned int samples;
    if ( !arguments.read("--samples", samples) )
        samples = 5;

    osg::ref_ptr<osg::Node> root = osgDB::readNodeFiles(arguments);
    if ( !root )
    {
        std::cout << "No model loaded." << std::endl;
        std::cout << arguments.getApplicationUsage()->getCommandLineUsage() << std::endl;
        arguments.getApplicationUsage()->write( std::cout, arguments.getApplicationUsage()->getCommandLineOptions() );
        return 1;
    }

    ConvertToBspVisitor bspVisitor( samples );
    root->accept( bspVisitor );

    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    return viewer.run();
}
