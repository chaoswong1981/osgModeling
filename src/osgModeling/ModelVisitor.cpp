/* -*-c++-*- osgModeling - Copyright (C) 2008 Wang Rui <wangray84@gmail.com>
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

#include <iostream>
#include <algorithm>
#include <osg/TriangleFunctor>
#include <osgModeling/Utilities>
#include <osgModeling/Model>
#include <osgModeling/ModelVisitor>
#include <osgModeling/PolyMesh>
#include <osgModeling/BspTree>

using namespace osgModeling;

struct CalcTriangleFunctor
{
    typedef std::multiset<const osg::Vec3*, LessPtr> CoordinateSet;
    ModelVisitor::GeometryTask _task;
    unsigned int _coordSize;
    CoordinateSet _coordSet;
    osg::Vec3Array* _coordArray;

    // Polymesh building variables & functions.
    typedef std::pair<CoordinateSet::iterator, CoordinateSet::iterator> EqualGroup;
    PolyMesh::EdgeMap* _meshEdges;
    PolyMesh::FaceList* _meshFaces;

    void setMeshPtr( PolyMesh::EdgeMap* em, PolyMesh::FaceList* fl )
    {
        _meshEdges = em;
        _meshFaces = fl;
    }

    inline void buildEdge( EqualGroup g1, EqualGroup g2, PolyMesh::Face* face )
    {
        for ( CoordinateSet::iterator itr1=g1.first; itr1!=g1.second; ++itr1 )
        {
            osg::Vec3 p1 = *(*itr1);
            for ( CoordinateSet::iterator itr2=g2.first; itr2!=g2.second; ++itr2 )
            {
                osg::Vec3 p2 = *(*itr2);
                PolyMesh::Segment p( p1, p2 );
                if ( p2<p1 )
                {
                    p.first = p2;
                    p.second = p1;
                }

                if ( _meshEdges->find(p)==_meshEdges->end() )
                    (*_meshEdges)[p] = new PolyMesh::Edge( p.first, p.second );
                (*_meshEdges)[p]->hasFace( face, true );
            }
        }
    }

    inline void buildMesh( const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3 )
    {
        osg::Vec3* cb = &(_coordArray->front());
        int p1=&v1-cb, p2=&v2-cb, p3=&v3-cb;
        PolyMesh::Face* face =  new PolyMesh::Face( _coordArray, p1, p2, p3 );
        _meshFaces->push_back( face );

        EqualGroup g1=_coordSet.equal_range(&v1),
            g2=_coordSet.equal_range(&v2),
            g3=_coordSet.equal_range(&v3);
        buildEdge( g1, g2, face );
        buildEdge( g2, g3, face );
        buildEdge( g3, g1, face ); 
    }

    // BSP faces building variables & functions.
    BspTree* _bspTree;

    void setBspPtr( BspTree* bsp )
    {
        _bspTree = bsp;
    }

    // General functions.
    CalcTriangleFunctor():
        _coordSize(0), _coordArray(0)
    {}

    void setTask( ModelVisitor::GeometryTask t ) { _task=t; }

    void setVerticsPtr( osg::Vec3Array* ca, unsigned int cs )
    {
        _coordSize = cs;
        _coordArray = ca;

        osg::Vec3* vptr = &(ca->front());
        for ( unsigned int i=0; i<cs; ++i )
            _coordSet.insert( vptr++ );
    }

    inline void operator() ( const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool treatVertexDataAsTemporary )
    {
        if ( treatVertexDataAsTemporary || v1==v2 || v1==v3 || v2==v3 )
            return;
        
        if ( _task==ModelVisitor::BUILD_MESH )
        {
            buildMesh( v1, v2, v3 );
        }
        else if ( _task==ModelVisitor::BUILD_BSP )
        {
            BspTree::BspFace face;
            face.addPoint( v1 );
            face.addPoint( v2 );
            face.addPoint( v3 );

            bool hasNormal;
            calcNormal( v1, v2, v3, &hasNormal );

            if ( face.valid() && hasNormal )
                _bspTree->addFace( face );
        }
    }
};

ModelVisitor::ModelVisitor()
{
    setTraversalMode( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
}

ModelVisitor::~ModelVisitor()
{
}

bool ModelVisitor::checkPrimitives( osg::Geometry& geom )
{
    osg::Geometry::PrimitiveSetList& primitives = geom.getPrimitiveSetList();
    osg::Geometry::PrimitiveSetList::iterator itr;
    unsigned int numSurfacePrimitives=0;
    for ( itr=primitives.begin(); itr!=primitives.end(); ++itr )
    {
        switch ( (*itr)->getMode() )
        {
        case (osg::PrimitiveSet::TRIANGLES):
        case (osg::PrimitiveSet::TRIANGLE_STRIP):
        case (osg::PrimitiveSet::TRIANGLE_FAN):
        case (osg::PrimitiveSet::QUADS):
        case (osg::PrimitiveSet::QUAD_STRIP):
        case (osg::PrimitiveSet::POLYGON):
            ++numSurfacePrimitives;
            break;
        default:
            break;
        }
    }

    if ( !numSurfacePrimitives ) return false;
    return true;
}

void ModelVisitor::buildBSP( Model& model )
{
    if ( !checkPrimitives(model) ) return;

    BspTree* bsp = model.getBspTree();
    osg::Vec3Array *coords = dynamic_cast<osg::Vec3Array*>( model.getVertexArray() );
    if ( !bsp || !coords || !coords->size() ) return;

    osg::TriangleFunctor<CalcTriangleFunctor> ctf;
    ctf.setTask( BUILD_BSP );
    ctf.setVerticsPtr( coords, coords->size() );
    ctf.setBspPtr( bsp );
    model.accept( ctf );

    bsp->buildBspTree();
}

void ModelVisitor::buildMesh( PolyMesh& mesh )
{
    if ( !checkPrimitives(mesh) ) return;

    osg::Vec3Array* coords = dynamic_cast<osg::Vec3Array*>( mesh.getVertexArray() );
    if ( !coords || !coords->size() ) return;

    osg::TriangleFunctor<CalcTriangleFunctor> ctf;
    ctf.setTask( BUILD_MESH );
    ctf.setVerticsPtr( coords, coords->size() );
    ctf.setMeshPtr( &(mesh._edges), &(mesh._faces) );
    mesh.accept( ctf );
}

void ModelVisitor::apply(osg::Geode& geode)
{
    for(unsigned int i = 0; i < geode.getNumDrawables(); i++ )
    {
        if ( _task==BUILD_BSP )
        {
            Model* model = dynamic_cast<Model*>( geode.getDrawable(i) );
            if ( model ) buildBSP( *model );
        }
        else if ( _task==BUILD_MESH )
        {
            PolyMesh* mesh = dynamic_cast<PolyMesh*>( geode.getDrawable(i) );
            if ( mesh ) buildMesh( *mesh );
        }
    }
}
