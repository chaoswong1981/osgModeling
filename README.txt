osgModeling is a open source modeling library for OpenSceneGraph(OSG).
Its purpose is to help generate kinds of parametric curves and surfaces,
and calculate vertices, normals and texture coordnates automatically.

For up-to-date information and latest versions of the project, please visit:

    http://code.google.com/p/osgmodeling/

For in-depth details of OpenSceneGraph, please visit its website:

    http://www.openscenegraph.org

===============================
Requirements:
===============================

- OpenSceneGraph: version 2.6.0 or higher

===============================
Features:
===============================

- Support for 4 kinds of curves:
  * k-degree Bezier curves.
  * k-degree NURBS curves.
  * Helix (3-dimensional spiral curves resembling a spring).
  * And user customized curves.

- Support for 6 kinds of surfaces:
  * m,n-degree Bezier surfaces.
  * m,n-degree NURBS surfaces.
  * Extrusions (constructed by a profile extruded along a path).
  * Revolutions (constructed by a profile rotated specified angles).
  * Lofts (constructed by lofting a series of curves that define the cross section on a specified path).
  * And user customized models.

- Generate normal arrays and texture coordinate arrays for various models (except user customizations).
  * Support 6 methods to generate normals with different weights.
  * Support the normal-flip operation.

- Free to define customized algorithms to created vertices arrays, normal arrays and texture coordinate arrays for own models.

- Construct the polygon mesh structures (Vertices-Edges-Faces) for geometries.

- Subdivide polygon meshes into higher level using different methods.
  * Loop method: Split each face into 4 parts at every level to build subdivisions.
  * Sqrt(3) method: Split each face into 3 parts at every level to build subdivisions.

- Construct the binary space partitioning (BSP) trees for models in built or converted from osg::Geometry.

- Geometric boolean operations (Intersection, Union and Difference) based on BSP trees of models.

===============================
How to build:
===============================

The osgModeling uses the CMake build system to generate a platform-specific build environment.
If you don't have CMake version 2.4.6 or later installed, you may download it from:
    
    http://www.cmake.org

Under unices, type the following commands:

    ./configure
    make
    sudo make install

Under Windows, just use the CMake GUI tool to build Visual Studio solutions.

===============================
Libraries and examples:
===============================

There are 1 or 2 dynamic library file after installation:

- {libosgModeling.so | osgModeling.dll}: The library.

- {libosgdb_osgmodeling.so | osgdb_osgmodeling.dll}: The reader/writer plugin.

There are 5 executable files as examples:

- osgmodelingbasic.exe: Demonstrates how to build extrusions, revolutions and lofts with osgModeling
classes (Extrude, Lathe, Loft, Helix and so on).

- osgmodelingboolean.exe: Demonstrates how to create a boolean object from two specified models.

- osgmodelingbsptree.exe: Demonstrates how to build BSP trees for models.

- osgmodelingnurbs.exe: Demonstrates how to create Bezier and NURBS surfaces with osgModeling classes.

- osgmodelingsubd.exe: Demonstrates how to subdivide models using different levels and methods (Loop or Sqrt3).

===============================
Bibliography:
===============================

[1] Philip Schneider, David H. Eberly, Geometric Tools for Computer Graphics, Elsevier Science, 2002
[2] Tomas Akenine-Moller, Eric Haines, Real-Time Rendering, A.K.Peters, 2002
[3] Piegl, Tiller, The NURBS Book, Springer, 1997
[4] Denis Zorin, Peter Schroder, Subdivision for Modeling and Animation, SIGGRAPH 2000 Course Notes, 2000
[5] Charles T. Loop, Smooth Subdivision Surfaces Based on Triangles, Utah University, 1987
[6] Leif Kobbelt, Sqrt3-Subdivision, Max-Planck Institute for Computer Sciences, 2000
[7] Shuangshuang Jin, Robert R. Lewis, David West, "A Comparison of Algorithms for Vertex Normal Computation", Washington State University, 2003
