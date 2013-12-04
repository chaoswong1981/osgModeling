/* -*-c++-*- osgModeling Example: Basic modeling tools
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

#include <osgModeling/Helix>
#include <osgModeling/Bezier>
#include <osgModeling/Extrude>
#include <osgModeling/Lathe>
#include <osgModeling/Loft>

const double xInterval = 4.0f;

// User customized algorithm callback, to calculate a sinusoid on XOY.
// The form is: y(t) = A * sin(wt). A means amplitude and w is the angular frequency.
class SinusoidCallback : public osgModeling::AlgorithmCallback
{
public:
    SinusoidCallback( double w, double a, double tmin, double tmax, unsigned int numPath ):
        _freq(w), _amplitude(a), _tMin(tmin), _tMax(tmax), _numPath(numPath) {}

    // This is the most important function to be implemented for a curve.
    // It you wish to create a model using an algorithm callback, just rewrite the other overloading.
    virtual void operator()( osgModeling::Curve* c )
    {
        osg::ref_ptr<osg::Vec3Array> pathArray = new osg::Vec3Array;

        double interval = (_tMax - _tMin) / (double)(_numPath-1);
        for ( unsigned int i=0; i<_numPath; ++i )
        {
            double t = _tMin + interval*i;
            pathArray->push_back( osg::Vec3(t, _amplitude*sin(t*_freq), 0.0f) );
        }
        c->setPath( pathArray.get() );
    }

protected:
    double _freq;
    double _amplitude;
    double _tMin;
    double _tMax;
    unsigned int _numPath;
};

osg::ref_ptr<osg::Node> createExtrusion()
{
    // First, we create a simple extrusion with a triangular profile and depth 2.0f.
    // The model is generated along the Z- axis and with top & bottom caps.
    // User should call update() before the main loop to create vertices for the model.
    osg::ref_ptr<osg::Vec3Array> ptArray = new osg::Vec3Array;
    ptArray->push_back( osg::Vec3( 0.0f-xInterval,-1.0f, 0.0f) );
    ptArray->push_back( osg::Vec3( 1.0f-xInterval, 1.0f, 0.0f) );
    ptArray->push_back( osg::Vec3(-1.0f-xInterval, 1.0f, 0.0f) );
    ptArray->push_back( osg::Vec3( 0.0f-xInterval,-1.0f, 0.0f) );
    osg::ref_ptr<osgModeling::Curve> profile1 = new osgModeling::Curve;
    profile1->setPath( ptArray.get() );
    osg::ref_ptr<osgModeling::Extrude> geom1 = new osgModeling::Extrude;
    geom1->setExtrudeDirection( osg::Vec3(0.0f, 0.0f, -1.0f) );
    geom1->setExtrudeLength( 2.0f );
    geom1->setProfile( profile1.get() );
    geom1->update();

    // The texture should be applied after generating the model.
    // The texcoords of model is limited in (0.0,0.0)-(1.0,1.0) of the texture map.
    // The extruding direction corresponds to the T direction of texture, and the unfolded profile
    // will be mapped on the S direction.
    geom1->getOrCreateStateSet()->setTextureAttributeAndModes(
        0, new osg::Texture2D(osgDB::readImageFile("osg_banner.jpg")) );

    // Second, a pentagram is built. Profile data is read from a double array.
    // This time, a convenient constructor is used to specify parameters for extrusion.
    // The model is along the Y- axis with the depth 0.2f, and scaled to 2/5 of origin model.
    double pentagram[11][3] = {
        {1.10f, 0.0f, -0.0f},  {1.34f, 0.0f, -0.72f}, {2.18f, 0.0f, -0.72f},
        {1.46f, 0.0f, -1.08f}, {1.66f, 0.0f, -1.92f}, {1.10f, 0.0f, -1.44f},
        {0.54f, 0.0f, -1.92f}, {0.74f, 0.0f, -1.08f}, {0.02f, 0.0f, -0.72f},
        {0.86f, 0.0f, -0.72f}, {1.10f, 0.0f, -0.0f} };
    osg::ref_ptr<osgModeling::Curve> profile2 = new osgModeling::Curve;
    profile2->setPath( 33, &pentagram[0][0] );
    osg::ref_ptr<osgModeling::Extrude> geom2 =
        new osgModeling::Extrude( profile2.get(), 0.2f, 0.4f, osg::Vec3(0.0f, -1.0f, 0.0f) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom1.get() );
    geode->addDrawable( geom2.get() );
    return geode;
}

osg::ref_ptr<osg::Node> createLathe()
{
    // First, we create a simple cylinder with caps.
    // Default segments is 16 and default rotating angle is 360 degree.
    osg::ref_ptr<osgModeling::Curve> profile1 = new osgModeling::Curve;
    profile1->addPathPoint( osg::Vec3(-0.5f, 0.0f, 0.0f) );
    profile1->addPathPoint( osg::Vec3(-0.5f, 0.0f, 2.0f) );
    osg::ref_ptr<osgModeling::Lathe> geom1 = new osgModeling::Lathe;

    // Here we change the 'GenerateParts' property to 'ALL_PARTS'.
    // Default is BODY_PART' only.
    geom1->setGenerateParts( osgModeling::Model::ALL_PARTS );

    // Note that the lathe-origin vector won't affect the profile and the rotate axis.
    geom1->setLatheOrigin( osg::Vec3(-xInterval, 0.0f, 0.0f) );

    geom1->setLatheAxis( osg::Vec3(0.0f, 0.0f, 1.0f) );
    geom1->setProfile( profile1.get() );
    geom1->update();

    // The texcoords of model is limited in (0.0,0.0)-(1.0, 0.5) if have caps.
    // Top cap is in (0.0,0.5)-(0.5, 1.0), and bottom cap in (0.5,0.5)-(1.0, 1.0).
    // For a revolution, the contour (profile) will be mapped on the T direction of texture,
    // and the rotating segments on S direction.
    geom1->getOrCreateStateSet()->setTextureAttributeAndModes(
        0, new osg::Texture2D(osgDB::readImageFile("osg_banner2.jpg")) );

    // Second, we will create a bottle-like model with a Bezier curve as profile.
    double cp[4][3] = {
        {-0.4f, 0.0f, 0.0f}, {-0.5f, 0.0f, 2.0f},
        {-0.2f, 0.0f, 0.5f}, {-0.1f, 0.0f, 2.0f} };
    osg::ref_ptr<osgModeling::BezierCurve> bezCurve =
        new osgModeling::BezierCurve( 3, 4, &cp[0][0] );
    osg::ref_ptr<osgModeling::Lathe> geom2 = new osgModeling::Lathe( bezCurve.get(),
        16, 2*osg::PI, osg::Vec3(1.0f,0.0f,0.0f), osg::Vec3(0.0f,0.0f,-1.0f) );

    // Sometimes we should modify the model, just update() again to rebuild it.
    // Note that the normals of the model may not be correct because of different rotate axises.
    // In that case, we may change the axis and profile, or simply use FLIP_NORMAL and rebuild the model.
    geom2->setAuxFunctions( osgModeling::Model::FLIP_NORMAL );
    geom2->setGenerateParts( osgModeling::Model::BODY_PART|osgModeling::Model::CAP1_PART );
    geom2->update();

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom1.get() );
    geode->addDrawable( geom2.get() );
    return geode;
}

osg::ref_ptr<osg::Node> createLoft()
{
    // First, we would create a helicoid geometry. To finish the work with Loft class, we should have
    // at least a helix path and a cross-section shape. We may just use the Helix class to generate
    // the required curve (Need to set number of coils, and radius and pitch parameters).
    osg::ref_ptr<osgModeling::Helix> helix =
        new osgModeling::Helix( 1.5f, 0.2f, 1.0f, osg::Vec3(-xInterval, 0.0f, 0.0f) );

    // And then, set the section shape and a loft model is created. The simplest section which is only a line segment
    // will be applied to the whole model. Loft class suggest use 2D curves (on XOY plane) as section shapes.
    osg::ref_ptr<osgModeling::Curve> section = new osgModeling::Curve;
    section->addPathPoint( osg::Vec3(0.5f,0.0f,0.0f) );
    section->addPathPoint( osg::Vec3(-0.5f,0.0f,0.0f) );
    osg::ref_ptr<osgModeling::Loft> geom1 =
        new osgModeling::Loft( helix.get(), section.get() );

    // Second, we plan to create a curtain in the 3d space. Usually we open a 3d modeling software like 3dsmax and Maya
    // to finish the model and display it with the scene graph. But here we could use the Loft class to do that.
    // The loft path is simple this time.
    osg::ref_ptr<osgModeling::Curve> path = new osgModeling::Curve;
    path->addPathPoint( osg::Vec3(0.0f,0.0f,2.0f) );
    path->addPathPoint( osg::Vec3(0.0f,0.0f,-1.0f) );

    // Next we would create the section shape for the loft object. This time a customized algorithm is used to generate
    // sine waves on a 2D plane. For a curtain, two kinds of sections are always needed: the top of the curtain is
    // planar, and the bottom is like a wave.
    // Note that the size of two section curves should be equal, otherwise the model may be distorted. This is the
    // reason why the SinusoidCallback is used twice (and the amplitude is set to 0 to make one curve straight).
    osg::ref_ptr<osgModeling::Curve> section1 = new osgModeling::Curve;
    section1->setAlgorithmCallback( new SinusoidCallback(4.0f, 0.0f, 0.0f, osg::PI, 36) );
    section1->update();
    osg::ref_ptr<osgModeling::Curve> section2 = new osgModeling::Curve;
    section2->setAlgorithmCallback( new SinusoidCallback(4.0f, 0.5f, 0.0f, osg::PI, 36) );
    section2->update();

    // The section shape should be attached to a specified position of the path. For example, the path with 2 points may
    // have at most 2 shapes attached. And a linear interpolation will be made for transitions.
    osg::ref_ptr<osgModeling::Loft> geom2 = new osgModeling::Loft;
    geom2->setProfile( path.get() );
    geom2->addShape( section1.get() );
    geom2->addShape( section2.get() );
    geom2->update();

    // The image file should flip tp fit the texture coordinates here, because points on 'section1' are placed
    // at 't=0.0f' of the texture and 'section2' at 't=1.0f' of the texture map. The unfolded section shape will be
    // mapped on the S direction for auto-generated texcoords.
    geom2->getOrCreateStateSet()->setTextureAttributeAndModes(
        0, new osg::Texture2D(osgDB::readImageFile("curtain_flip.jpg")) );

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable( geom1.get() );
    geode->addDrawable( geom2.get() );
    return geode;
}

int main( int argc, char** argv )
{
    osg::ref_ptr<osg::PositionAttitudeTransform> extrusions = new osg::PositionAttitudeTransform;
    extrusions->setPosition( osg::Vec3(0.0f, 0.0f, 5.0f) );
    extrusions->addChild( createExtrusion().get() );

    osg::ref_ptr<osg::PositionAttitudeTransform> revolutions = new osg::PositionAttitudeTransform;
    revolutions->setPosition( osg::Vec3(0.0f, 0.0f, -1.0f) );
    revolutions->addChild( createLathe().get() );

    osg::ref_ptr<osg::PositionAttitudeTransform> lofts = new osg::PositionAttitudeTransform;
    lofts->setPosition( osg::Vec3(0.0f, 0.0f, -5.0f) );
    lofts->addChild( createLoft().get() );

    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild( extrusions.get() );
    root->addChild( revolutions.get() );
    root->addChild( lofts.get() );

    osgViewer::Viewer viewer;
    viewer.setSceneData( root );
    return viewer.run();
}
