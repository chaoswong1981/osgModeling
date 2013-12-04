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

#include <osgModeling/Utilities>
#include <osgModeling/Subdivision>

using namespace osgModeling;

void Subdivision::operator()( PolyMesh* mesh )
{
    for ( int i=0; i<_level; ++i )
        subdivide( mesh );
    PolyMesh::convertFacesToGeometry( mesh->_faces, mesh );
}

LoopSubdivision::LoopSubdivision( int level ):
    Subdivision()
{
    setLevel( level );
}

LoopSubdivision::LoopSubdivision( const LoopSubdivision& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    Subdivision(copy, copyop)
{
}

LoopSubdivision::~LoopSubdivision()
{
}

void LoopSubdivision::subdivide( PolyMesh* mesh )
{
    if ( !mesh || !mesh->_faces.size() ) return;

    PolyMesh::MeshType type = mesh->getType();
    if ( type==PolyMesh::INVALID_MESH || type==PolyMesh::NONMANIFOLD_MESH )
        return;

    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>( mesh->getVertexArray() );
    if ( !vertices || !vertices->size() ) return;
    unsigned int ptNum = vertices->size();

    // Reset current vertices to build subdivision points.
    // Here we use a reference array and put modified vertices in it first.
    // The modified vertices will be used for point-edge map (PolyMesh::EdgeMap) generating,
    // BUT original ones should be used for edge splitting.
    osg::ref_ptr<osg::Vec3Array> refVertices = new osg::Vec3Array( vertices->begin(), vertices->end() );
    subdivideVertices( mesh, refVertices.get(), ptNum );

    // Split edges to build subdivision points.
    // New allocated edge points will be inserted into the 'vertices' array, and 'refVertices'
    // is used for point-edge map generating.
    subdivideFace( mesh, mesh->_faces.front(), refVertices.get(), vertices );

    // Merge 'refVertices' points, which were modified just now, into 'vertices'
    vertices->erase( vertices->begin(), vertices->begin()+ptNum );
    vertices->insert( vertices->begin(), refVertices->begin(), refVertices->end() );

    mesh->destroyMesh();
    mesh->_edges.swap( _tempEdges );
    mesh->_faces.swap( _tempFaces );
    _edgeVertices.clear();
    _tempEdges.clear();
    _tempFaces.clear();
}

void LoopSubdivision::subdivideVertices( PolyMesh* mesh, osg::Vec3Array* pts, unsigned int ptNum )
{
    for ( unsigned int i=0; i<ptNum; ++i )
    {
        PolyMesh::VertexList vlist;
        osg::Vec3 vec = (*pts)[i];
        mesh->findNeighbors( vec, vlist );

        unsigned int size = vlist.size();
        if ( size>2 )
        {
            osg::Vec3 summaryVec( 0.0f, 0.0f, 0.0f );
            for ( PolyMesh::VertexList::iterator itr=vlist.begin(); itr!=vlist.end(); ++itr )
                summaryVec += *itr;

            double beta = (size==3)?0.1875f:(0.375f/size);
            (*pts)[i] = vec*(1-size*beta) + summaryVec*beta;
        }
        else if ( size==2 )
        {
            (*pts)[i] = vec*0.75f + (vlist[0]+vlist[1])*0.125f;
        }
    }
}

void LoopSubdivision::subdivideFace( PolyMesh* mesh, PolyMesh::Face* f, osg::Vec3Array* refPts, osg::Vec3Array* pts )
{
    if ( !f || f->_flag ) return;

    int evIndex[3] = {0}; // Only for triangles
    osg::Vec3 edgeVertex[3];
    PolyMesh::EdgeList edges;
    PolyMesh::EdgeList::iterator itr;
    mesh->findEdgeList( f, edges );

    int i=0;
    for ( itr=edges.begin(); itr!=edges.end() && i<3; ++i, ++itr )
    {
        PolyMesh::Edge* edge = *itr;
        if ( _edgeVertices.find(edge)!=_edgeVertices.end() )
        {
            evIndex[i] = _edgeVertices[edge];
            edgeVertex[i] = pts->at( evIndex[i] );
        }
        else
        {
            osg::Vec3 ev0=(*edge)[0], ev1=(*edge)[1];
            PolyMesh::FaceList fl = edge->_faces;
            if ( fl.size()<2 )
            {
                // Boundary edges
                edgeVertex[i] = (ev0+ev1) * 0.5f;
            }
            else
            {
                // Interior edges
                osg::Vec3 v1=*(fl[0])-(*edge), v2=*(fl[1])-(*edge);
                edgeVertex[i] = (ev0+ev1)*0.375f + (v1+v2)*0.125f;
            }

            pts->push_back( edgeVertex[i] );
            evIndex[i] = pts->size()-1;
            _edgeVertices[edge] = evIndex[i];
        }
    }

    // Construct new edges and triangles
    PolyMesh::Face* newFace[4];
    newFace[0] = new PolyMesh::Face( f->_array, (*f)(0), evIndex[0], evIndex[2] );
    newFace[1] = new PolyMesh::Face( f->_array, (*f)(1), evIndex[1], evIndex[0] );
    newFace[2] = new PolyMesh::Face( f->_array, (*f)(2), evIndex[2], evIndex[1] );
    newFace[3] = new PolyMesh::Face( f->_array, evIndex[0], evIndex[1], evIndex[2] );
    for ( i=0; i<4; ++i )
    {
        PolyMesh::buildEdges( newFace[i], refPts, _tempEdges );
        _tempFaces.push_back( newFace[i] );
    }

    // Traverse to neighbor faces
    f->_flag = 1;   // Sign this face as 'visited'
    PolyMesh::FaceList flist;
    mesh->findNeighbors( f, flist );
    for ( PolyMesh::FaceList::iterator fitr=flist.begin(); fitr!=flist.end(); ++fitr )
        subdivideFace( mesh, *fitr, refPts, pts );
}

Sqrt3Subdivision::Sqrt3Subdivision( int level ):
    Subdivision()
{
    setLevel( level );
}

Sqrt3Subdivision::Sqrt3Subdivision( const Sqrt3Subdivision& copy, const osg::CopyOp& copyop/*=osg::CopyOp::SHALLOW_COPY*/ ):
    Subdivision(copy, copyop)
{
}

Sqrt3Subdivision::~Sqrt3Subdivision()
{
}

void Sqrt3Subdivision::subdivide( PolyMesh* mesh )
{
    if ( !mesh || !mesh->_faces.size() ) return;

    PolyMesh::MeshType type = mesh->getType();
    if ( type==PolyMesh::INVALID_MESH || type==PolyMesh::NONMANIFOLD_MESH )
        return;

    osg::Vec3Array* vertices = dynamic_cast<osg::Vec3Array*>( mesh->getVertexArray() );
    if ( !vertices || !vertices->size() ) return;
    unsigned int ptNum = vertices->size();

    osg::ref_ptr<osg::Vec3Array> refVertices = new osg::Vec3Array( vertices->begin(), vertices->end() );
    subdivideVertices( mesh, refVertices.get(), ptNum );
    subdivideFace( mesh, mesh->_faces.front(), refVertices.get(), vertices );
    vertices->erase( vertices->begin(), vertices->begin()+ptNum );
    vertices->insert( vertices->begin(), refVertices->begin(), refVertices->end() );

    // Spin every original edges in the new edge map
    for ( PolyMesh::EdgeMap::iterator eitr=_tempEdges.begin(); eitr!=_tempEdges.end(); )
    {
        if ( !eitr->second->_flag )
        {
            ++eitr;
            continue;
        }
        eitr->second->_flag = 0;
        PolyMesh::spinEdge( eitr, _tempEdges );
    }

    mesh->destroyMesh();
    mesh->_edges.swap( _tempEdges );
    mesh->_faces.swap( _tempFaces );
    _tempEdges.clear();
    _tempFaces.clear();
}

void Sqrt3Subdivision::subdivideVertices( PolyMesh* mesh, osg::Vec3Array* pts, unsigned int ptNum )
{
    for ( unsigned int i=0; i<ptNum; ++i )
    {
        PolyMesh::VertexList vlist;
        osg::Vec3 vec = (*pts)[i];
        mesh->findNeighbors( vec, vlist );

        unsigned int size = vlist.size();
        if ( !size ) continue;
        
        osg::Vec3 summaryVec( 0.0f, 0.0f, 0.0f );
        for ( PolyMesh::VertexList::iterator itr=vlist.begin(); itr!=vlist.end(); ++itr )
            summaryVec += *itr;

        double beta = (4-2*cos(2*osg::PI/size)) / (9*size);
        (*pts)[i] = vec*(1-size*beta) + summaryVec*beta;
    }
}

void Sqrt3Subdivision::subdivideFace( PolyMesh* mesh, PolyMesh::Face* f, osg::Vec3Array* refPts, osg::Vec3Array* pts )
{
    if ( !f || f->_flag ) return;

    // Create new point, only for triangles
    osg::Vec3 newVec = ((*f)[0]+(*f)[1]+(*f)[2]) / 3.0f;
    pts->push_back( newVec );
    int newIndex = pts->size()-1;

    // Construct new edges and triangles
    PolyMesh::Face* newFace[4];
    newFace[0] = new PolyMesh::Face( f->_array, (*f)(0), (*f)(1), newIndex );
    newFace[1] = new PolyMesh::Face( f->_array, (*f)(1), (*f)(2), newIndex );
    newFace[2] = new PolyMesh::Face( f->_array, (*f)(2), (*f)(0), newIndex );
    for ( int i=0; i<3; ++i )
    {
        PolyMesh::buildEdges( newFace[i], refPts, _tempEdges );
        _tempFaces.push_back( newFace[i] );
    }

    // Mark original edges for spinning later
    for ( unsigned int i=0; i<3; ++i )
    {
        PolyMesh::Edge* edge =
            PolyMesh::getEdge( (*refPts)[(*f)(i%3)], (*refPts)[(*f)((i+1)%3)], _tempEdges );
        if ( edge ) edge->_flag = 1;
    }

    // Traverse to neighbor faces
    f->_flag = 1;   // Sign this face as 'visited'
    PolyMesh::FaceList flist;
    mesh->findNeighbors( f, flist );
    for ( PolyMesh::FaceList::iterator fitr=flist.begin(); fitr!=flist.end(); ++fitr )
        subdivideFace( mesh, *fitr, refPts, pts );
}

