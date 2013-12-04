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

#include <map>
#include <osgModeling/Utilities>
#include <osgModeling/BspTree>
#include <osgModeling/BoolOperator>
#include <osgModeling/ModelVisitor>
#include <osgModeling/NormalVisitor>

using namespace osgModeling;

BoolOperator::BoolOperator( Method m ):
    osg::Object(),
    _method(m), _operand1(0), _operand2(0)
{
}

BoolOperator::BoolOperator( const BoolOperator& copy, const osg::CopyOp& copyop ):
    osg::Object(copy,copyop),
    _method(copy._method), _operand1(copy._operand1), _operand2(copy._operand2)
{
}

BoolOperator::~BoolOperator()
{
}

void BoolOperator::setOperands( Model* model1, Model* model2 )
{
    if ( !model1 || !model2 ) return;

    if ( !model1->getBspTree() )
    {
        model1->setBspTree( new osgModeling::BspTree );
        osgModeling::ModelVisitor::buildBSP( *model1 );
    }
    if ( !model2->getBspTree() )
    {
        model2->setBspTree( new osgModeling::BspTree );
        osgModeling::ModelVisitor::buildBSP( *model2 );
    }

    setOperands( model1->getBspTree(), model2->getBspTree() );
}

bool BoolOperator::output( osg::Geometry* result )
{
    if ( !_operand1 || !_operand2 ) return false;
    if ( !_operand1->getRoot() || !_operand2->getRoot() ) return false;

    // Receive data according to the boolean method.
    BspNode* op1 = _operand1->getRoot();
    BspNode* op2 = _operand2->getRoot();
    FaceList op1Faces = _operand1->getFaceList();
    FaceList op2Faces = _operand2->getFaceList();
    if ( _method==BOOL_UNION )
    {
        op1 = BspTree::reverseBspNode( _operand1->getRoot() );
        op1Faces = BspTree::reverseFaces( _operand1->getFaceList() );
        op2 = BspTree::reverseBspNode( _operand2->getRoot() );
        op2Faces = BspTree::reverseFaces( _operand2->getFaceList() );
    }
    else if ( _method==BOOL_DIFFERENCE )
    {
        op2 = BspTree::reverseBspNode( _operand2->getRoot() );
        op2Faces = BspTree::reverseFaces( _operand2->getFaceList() );
    }

    // Do intersecting operation of 2 objects.
    FaceList resultFaces;
    FaceList::iterator itr;
    for ( itr=op1Faces.begin(); itr!=op1Faces.end(); ++itr )
    {
        if ( _operand2->getBound().intersects(itr->getBound()) )
        {
            FaceList pos, neg, coinSame, coinNeg;
            _operand2->analyzeFace( op2, *itr, pos, neg, coinSame, coinNeg );
            resultFaces.insert( resultFaces.end(), neg.begin(), neg.end() );
            resultFaces.insert( resultFaces.end(), coinSame.begin(), coinSame.end() );
        }
        else if ( _method!=BOOL_INTERSECTION )
        {
            resultFaces.push_back( *itr );
        }
    }
    for ( itr=op2Faces.begin(); itr!=op2Faces.end(); ++itr )
    {
        if ( _operand1->getBound().intersects(itr->getBound()) )
        {
            FaceList pos, neg, coinSame, coinNeg;
            _operand1->analyzeFace( op1, *itr, pos, neg, coinSame, coinNeg );
            resultFaces.insert( resultFaces.end(), neg.begin(), neg.end() );
            //resultFaces.insert( resultFaces.end(), coinSame.begin(), coinSame.end() );
        }
        else if ( _method==BOOL_UNION )
        {
            resultFaces.push_back( *itr );
        }
    }

    // Do post operations.
    if ( _method==BOOL_UNION )
    {
        resultFaces = BspTree::reverseFaces( resultFaces );
        BspTree::destroyBspNode( op1 );
        BspTree::destroyBspNode( op2 );
    }
    else if ( _method==BOOL_DIFFERENCE )
    {
        BspTree::destroyBspNode( op2 );
    }

    convertFacesToGeometry( resultFaces, result );
    return true;
}

bool BoolOperator::convertFacesToGeometry( FaceList faces, osg::Geometry* geom )
{
    if ( !faces.size() || !geom ) return false;
    std::map<osg::Vec3, unsigned int> verticesMap;

    osg::ref_ptr<osg::Vec3Array> vertics = new osg::Vec3Array;
    osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt( osg::PrimitiveSet::TRIANGLES, 0 );

    unsigned int i, index=0, firstIndex=0, lastIndex=0;
    for ( FaceList::iterator itr=faces.begin();
        itr!=faces.end();
        ++itr )
    {
        BspFace f = *itr;
        unsigned int size=f._points.size();
        for ( i=0; i<size; ++i )
        {
            if ( i>2 )
            {
                indices->push_back( firstIndex );
                indices->push_back( lastIndex );
            }

            if ( verticesMap.find(f[i])==verticesMap.end() )
            {
                verticesMap[ f[i] ] = index;
                vertics->push_back( f[i] );
                indices->push_back( index++ );
            }
            else
            {
                indices->push_back( verticesMap[ f[i] ] );
            }

            if ( !i ) firstIndex = indices->back();
            lastIndex = indices->back();
        }
    }

    geom->removePrimitiveSet( 0, geom->getPrimitiveSetList().size() );
    geom->addPrimitiveSet( indices.get() );
    geom->setVertexArray( vertics.get() );
    geom->setTexCoordArray( 0, NULL );	// TEMP
    NormalVisitor::buildNormal( *geom );
    geom->dirtyDisplayList();
    return true;
}

void BoolOperator::triangulate( BspFace face, osg::Vec3 normal, FaceList& flist )
{
    unsigned int size=face._points.size();
    if ( size==3 )
    {
        flist.push_back( face );
        return;
    }

    unsigned int i0, i1, i2;
    for ( i0=0, i1=1, i2=2; i0<size;
        i0++, i1=(i1+1)%size, i2=(i2+1)%size )
    {
        if ( checkDiagonal(face, normal, i0, i2) )
        {
            BspFace newFace;
            newFace.addPoint( face[i0] );
            newFace.addPoint( face[i1] );
            newFace.addPoint( face[i2] );
            flist.push_back( newFace );

            newFace._points = face._points;
            newFace._points.erase( newFace._points.begin()+i1 );
            triangulate( newFace, normal, flist );
            return;
        }
    }
}

bool BoolOperator::checkDiagonal( BspFace face, osg::Vec3 normal, unsigned int i0, unsigned int ie )
{
    unsigned int size = face._points.size();
    int iPrev, iNext;
    if ( i0==0 ) iPrev = size-1;
    else iPrev = i0-1;
    iNext = (i0+1)%size;

    // Check if ie-i0 is in a cone formed by iPrev, i0 & iNext.
    osg::Vec3 vDiff=face[ie]-face[i0], eLeft=face[iPrev]-face[i0], eRight=face[iNext]-face[i0];
    osg::Vec3 cross=eLeft^eRight, crossLeft=vDiff^eLeft, crossRight=vDiff^eRight;
    cross.normalize();
    crossLeft.normalize();
    crossRight.normalize();
    if ( equivalent(cross,normal) )
    {
        if ( !equivalent(crossLeft,cross) && equivalent(crossRight,cross) )
            return false;
    }
    else
    {
        if ( equivalent(crossLeft,cross) || !equivalent(crossRight,cross) )
            return false;
    }

    // Test if there is intersection with other edges.
    bool ok;
    unsigned int j0, j1;
    for ( j0=0, j1=face._points.size()-1; j0<face._points.size(); j1=j0, j0++ )
    {
        if ( j0!=i0 && j0!=ie && j1!=i0 && j1!=ie )
        {
            calcIntersect( face[i0], vDiff, face[j0], face[j1]-face[j0], false, &ok );
            if ( ok ) return false;
        }
    }

    return true;
}
