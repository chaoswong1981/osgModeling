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
#include <osgModeling/ModelVisitor>
#include <osgModeling/BspTree>

using namespace osgModeling;

struct AddVecComparer
{
    osg::Vec3 _v;

    AddVecComparer( osg::Vec3 v ) { _v=v; }
    inline bool operator() ( const osg::Vec3 vec ) const
    {
        return equivalent( vec, _v );
    }
};

bool BspTree::BspFace::addPoint( osg::Vec3 p, bool replaceSame )
{
    if ( replaceSame )
    {
        BspTree::PointList::iterator rtn;
        rtn = std::find_if( _points.begin(), _points.end(), AddVecComparer(p) );
        if ( rtn!=_points.end() ) _points.erase( rtn );
    }

    _points.push_back( p );
    return true;
}

bool BspTree::BspFace::insertPoint( BspTree::PointList::iterator pos, osg::Vec3 p, bool ingoreSame )
{
    if ( ingoreSame )
    {
        BspTree::PointList::iterator rtn;
        rtn = std::find_if( _points.begin(), _points.end(), AddVecComparer(p) );
        if ( rtn!=_points.end() ) return false;
    }

    _points.insert( pos, p );
    return true;
}

void BspTree::BspFace::reverse()
{
    std::reverse( _points.begin(), _points.end() );
}

double BspTree::BspFace::orientation( osg::Vec3 refNormal )
{
    return valid() ?
        checkOrientation( _points[1]-_points[0], _points[2]-_points[0], refNormal ) : 0.0f; 
}

osg::BoundingBox BspTree::BspFace::getBound()
{
    osg::BoundingBox bound;
    for ( unsigned int i=0; i<_points.size(); ++i )
        bound.expandBy( _points[i] );
    return bound;
}

BspTree::BspTree( unsigned int numSearchBestDivider ):
    osg::Object(), _root(0), _numSearchBestDivider(numSearchBestDivider)
{
}

BspTree::BspTree( const BspTree& copy, const osg::CopyOp& copyop ):
    osg::Object(copy,copyop),
    _preFaces(copy._preFaces), _root(copy._root),
    _bound(copy._bound), _numSearchBestDivider(copy._numSearchBestDivider)
{
}

BspTree::~BspTree()
{
    destroyBspNode( _root );
}

void BspTree::buildBspTree()
{
    destroyBspNode( _root );
    _root = createBspNode( _preFaces );

    for ( FaceList::iterator itr=_preFaces.begin(); itr!=_preFaces.end(); ++itr )
        _bound.expandBy( (*itr).getBound() );
}

BspTree::BspNode* BspTree::createBspNode( FaceList fl )
{
    if ( !fl.size() || !fl.front().valid() ) return NULL;

    unsigned int selPos, i=0;
    BspFace selFace = findBestDivider( fl, selPos );
    osg::Plane plane = calcPlane( selFace[0], selFace[1], selFace[2] );
    BspNode* node = new BspNode( plane );
    FaceList posSubFaces, negSubFaces;
    for ( FaceList::iterator itr=fl.begin(); itr!=fl.end(); ++itr, ++i )
    {
        if ( i==selPos )
        {
            node->_coinFaces.push_back( *itr );
            continue;
        }

        BspFace posFace, negFace, currFace=*itr;
        FaceClassify type = partitionFace( node->_plane, currFace, posFace, negFace );
        switch ( type )
        {
        case CROSS_FACE:
            posSubFaces.push_back( posFace );
            negSubFaces.push_back( negFace );
            break;
        case POSITIVE_FACE:
            posSubFaces.push_back( currFace );
            break;
        case NEGATIVE_FACE:
            negSubFaces.push_back( currFace );
            break;
        case COINCIDENT_FACE:
            node->_coinFaces.push_back( currFace );
            break;
        case INVALID_FACE:
				break;
        }
    }

    node->_posChild = createBspNode( posSubFaces );
    node->_negChild = createBspNode( negSubFaces );
    return node;
}

BspTree::BspNode* BspTree::createBspNode2D( FaceList fl )
{
    if ( !fl.size() || !fl.front().valid() ) return NULL;

    BspFace firstFace = fl.front();
    osg::Vec3 s=firstFace._points.front(), e=firstFace._points.at(1);
    osg::Vec3 faceNormal = calcNormal(firstFace[0], firstFace[1], firstFace[2]);
    osg::Vec3 n = (e-s)^faceNormal;
    n.normalize();

    FaceList posSubFaces, negSubFaces;
    BspNode* node = new BspNode( osg::Plane(n, s) );
    for ( FaceList::iterator itr=fl.begin()+1; itr!=fl.end(); ++itr )
    {
        BspFace face = *itr;
        unsigned int i, size=face._points.size();
        for ( i=0; i<size; ++i )
        {
            BspFace posFace, negFace, currFace;
            currFace.addPoint( face[i] );
            currFace.addPoint( face[(i+1)%size] );
            currFace.addPoint( face[i]+faceNormal );

            FaceClassify type = partitionFace( node->_plane, currFace, posFace, negFace );
            switch ( type )
            {
            case CROSS_FACE:
                posSubFaces.push_back( posFace );
                negSubFaces.push_back( negFace );
                break;
            case POSITIVE_FACE:
                posSubFaces.push_back( currFace );
                break;
            case NEGATIVE_FACE:
                negSubFaces.push_back( currFace );
                break;
            case COINCIDENT_FACE:
                node->_coinFaces.push_back( currFace );
                break;
            case INVALID_FACE:
                break;
            }
        }
    }

    node->_posChild = createBspNode( posSubFaces );
    node->_negChild = createBspNode( negSubFaces );
    return node;
}

void BspTree::destroyBspNode( BspNode*& node )
{
    if ( !node ) return;

    destroyBspNode( node->_posChild );
    destroyBspNode( node->_negChild );
    delete node;
    node = 0;
}

BspTree::BspNode* BspTree::reverseBspNode( BspNode* node )
{
    if ( !node ) return NULL;

    osg::Plane originPlane = node->_plane;
    originPlane.flip();
    BspNode* negateNode = new BspNode( originPlane );
    negateNode->_coinFaces = reverseFaces( node->_coinFaces );

    if ( node->_posChild ) negateNode->_negChild = reverseBspNode( node->_posChild );
    else negateNode->_negChild = NULL;
    if ( node->_negChild ) negateNode->_posChild = reverseBspNode( node->_negChild );
    else negateNode->_posChild = NULL;
    return negateNode;
}

BspTree::FaceList BspTree::reverseFaces( FaceList fl )
{
    FaceList negateFaces;
    for ( FaceList::iterator itr=fl.begin(); itr!=fl.end(); ++itr )
    {
        BspFace face = *itr;
        face.reverse();
        negateFaces.push_back( face );
    }
    return negateFaces;
}

void BspTree::analyzeFace( BspNode* node, BspFace face, FaceList& posFaces, FaceList& negFaces,
                          FaceList& coinSame, FaceList& coinNeg )
{
    if ( !node || !face.valid() ) return;

    BspFace subPos, subNeg;
    FaceClassify type = partitionFace( node->_plane, face, subPos, subNeg );
    switch ( type )
    {
    case CROSS_FACE:
        if ( node->_posChild ) analyzeFace( node->_posChild, subPos, posFaces, negFaces, coinSame, coinNeg );
        else posFaces.push_back( subPos );
        if ( node->_negChild ) analyzeFace( node->_negChild, subNeg, posFaces, negFaces, coinSame, coinNeg );
        else negFaces.push_back( subNeg );
        break;
    case POSITIVE_FACE:
        if ( node->_posChild ) analyzeFace( node->_posChild, face, posFaces, negFaces, coinSame, coinNeg );
        else posFaces.push_back( face );
        break;
    case NEGATIVE_FACE:
        if ( node->_negChild ) analyzeFace( node->_negChild, face, posFaces, negFaces, coinSame, coinNeg );
        else negFaces.push_back( face );
        break;
    case COINCIDENT_FACE:
        {
            // Calculate the intersection and difference of current face & node-plane faces.
            FaceList posList, negList;
            FaceList::iterator itr;
            BspNode* root2D = createBspNode2D( node->_coinFaces );
            analyzeFace2D( root2D, face, posList, negList );
            destroyBspNode( root2D );

            // Add intersection of faces, which have same directions with the current face, to result.
            osg::Vec3 faceNormal = calcNormal( face[0], face[1], face[2] );
            for ( itr=negList.begin(); itr!=negList.end(); ++itr )
            {
                osg::Vec3 n = calcNormal( (*itr)[0], (*itr)[1], (*itr)[2] );
                if ( equivalent(node->_plane.getNormal(), faceNormal) ) coinSame.push_back( *itr );
                else coinNeg.push_back( *itr );
            }

            // Go on analyze difference faces.
            for ( itr=posList.begin(); itr!=posList.end(); ++itr )
            {
                if ( node->_posChild )
                    analyzeFace( node->_posChild, *itr, posFaces, negFaces, coinSame, coinNeg );
                else
                    posFaces.push_back( *itr );
                if ( node->_negChild )
                    analyzeFace( node->_negChild, *itr, posFaces, negFaces, coinSame, coinNeg );
                else
                    negFaces.push_back( *itr );
            }
        }
        break;
    case INVALID_FACE:
        break;
    }
}

void BspTree::analyzeFace2D( BspNode* node, BspFace face, FaceList& posFaces, FaceList& negFaces )
{
    if ( !node ) return;

    BspFace subPos, subNeg;
    FaceClassify type = partitionFace( node->_plane, face, subPos, subNeg );
    switch ( type )
    {
    case CROSS_FACE:
        if ( node->_posChild ) analyzeFace2D( node->_posChild, subPos, posFaces, negFaces );
        else posFaces.push_back( subPos );
        if ( node->_negChild ) analyzeFace2D( node->_negChild, subNeg, posFaces, negFaces );
        else negFaces.push_back( subNeg );
        break;
    case POSITIVE_FACE:
        if ( node->_posChild ) analyzeFace2D( node->_posChild, face, posFaces, negFaces );
        else posFaces.push_back( face );
        break;
    case NEGATIVE_FACE:
        if ( node->_negChild ) analyzeFace2D( node->_negChild, face, posFaces, negFaces );
        else negFaces.push_back( face );
        break;
    default:
        break;
    }
}

BspTree::FaceClassify BspTree::partitionFace( osg::Plane plane, BspFace face, BspFace& posFace, BspFace& negFace )
{
    osg::Vec3 lastPt;
    int lastPtState=0xff;  // 1 for pt+, -1 for pt-, 0 for coincident and 0xFF for undefined
    int posPt=0, negPt=0, coinPt=0;
    for ( unsigned int i=0; i<=face._points.size(); ++i )
    {
        osg::Vec3 vec = (i==face._points.size()) ? face[0] : face[i];
        double dis = plane.distance( vec );

        if ( osg::equivalent(dis,(double)0.0f) )
        {
            lastPtState = 0;
            coinPt++;
            posFace.addPoint( vec );
            negFace.addPoint( vec );
        }
        else if ( dis>0.0f )
        {
            if ( lastPtState<0 )
            {
                osg::Vec3 ip = calcIntersect( vec, vec-lastPt, plane );
                posFace.addPoint( ip );
                negFace.addPoint( ip );
            }

            lastPtState = 1;
            posPt++;
            posFace.addPoint( vec );
        }
        else
        {
            if ( lastPtState>0 && lastPtState!=0xff )
            {
                osg::Vec3 ip = calcIntersect( vec, vec-lastPt, plane );
                posFace.addPoint( ip );
                negFace.addPoint( ip );
            }

            lastPtState = -1;
            negPt++;
            negFace.addPoint( vec );
        }

        lastPt = vec;
    }

    if ( posPt>0 && negPt>0 ) return CROSS_FACE;
    else if ( posPt>0 ) return POSITIVE_FACE;
    else if ( negPt>0 ) return NEGATIVE_FACE;
    else if ( coinPt>0 ) return COINCIDENT_FACE;
    else return INVALID_FACE;
}

BspTree::BspFace BspTree::findBestDivider( FaceList fl, unsigned int& bestPos )
{
    BspFace* bestFace=&(fl.front());
    double bestRelation=0.0f;
    unsigned int leastCross=fl.size();
    bestPos = 0;

    unsigned int i=0, checkNum;
    if ( !_numSearchBestDivider || fl.size()<_numSearchBestDivider ) checkNum = 1;
    else checkNum = fl.size()/_numSearchBestDivider;

    for ( FaceList::iterator fitr=fl.begin();
        fitr!=fl.end() && (!_numSearchBestDivider || i<_numSearchBestDivider);
        fitr=fitr+checkNum, ++i )
    {
        BspFace simpleFace = *fitr;
        if ( !simpleFace.valid() ) continue;

        osg::Plane plane = calcPlane( simpleFace[0], simpleFace[1], simpleFace[2] );
        double relation=0.0f;
        unsigned int posNum=0, negNum=0, crossNum=0;

        for ( FaceList::iterator sitr=fl.begin(); sitr!=fl.end(); ++sitr )
        {
            if ( fitr==sitr ) continue;

            BspFace posFace, negFace, currFace=*sitr;
            FaceClassify type = partitionFace( plane, currFace, posFace, negFace );
            switch ( type )
            {
            case CROSS_FACE: crossNum++; break;
            case POSITIVE_FACE: posNum++; break;
            case NEGATIVE_FACE: negNum++; break;
            default: break;
            }
        }

        // Record a better face.
        if ( posNum>negNum ) relation = (double)negNum/(double)posNum;
        else if ( negNum ) relation = (double)posNum/(double)negNum;

        if ( relation>bestRelation && crossNum<=leastCross )
        {
            bestRelation = relation;
            leastCross = crossNum;
            bestFace = &(*fitr);
            bestPos = fitr-fl.begin();
        }
    }
    return *bestFace;
}
