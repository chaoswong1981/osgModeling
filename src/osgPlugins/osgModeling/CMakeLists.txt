SET(LIB_NAME osgdb_osgmodeling)

SET(SOURCES
    IO_Curve.cpp
    IO_Model.cpp
    IO_Extrude.cpp
    IO_Lathe.cpp
    IO_Loft.cpp
    IO_Helix.cpp
    IO_Bezier.cpp
    IO_Nurbs.cpp
    IO_BspTree.cpp
    IO_PolyMesh.cpp
)

ADD_LIBRARY(${LIB_NAME} SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(${LIB_NAME}
    debug osg${OSG_DEBUG_POSTFIX}         optimized osg
    debug osgUtil${OSG_DEBUG_POSTFIX}     optimized osgUtil
    debug osgViewer${OSG_DEBUG_POSTFIX}   optimized osgViewer
    debug osgText${OSG_DEBUG_POSTFIX}     optimized osgText
    debug osgDB${OSG_DEBUG_POSTFIX}       optimized osgDB
    debug osgGA${OSG_DEBUG_POSTFIX}       optimized osgGA
    debug OpenThreads${OSG_DEBUG_POSTFIX} optimized OpenThreads
    debug osgModeling${OSG_DEBUG_POSTFIX} optimized osgModeling
)

SET(INSTALL_BINDIR bin)
IF(WIN32)
    SET(INSTALL_LIBDIR bin)
    SET(INSTALL_ARCHIVEDIR lib)
ELSE(WIN32)
    SET(INSTALL_LIBDIR lib${LIB_POSTFIX})
    SET(INSTALL_ARCHIVEDIR lib${LIB_POSTFIX})
ENDIF(WIN32)

INSTALL(
    TARGETS ${LIB_NAME}
    RUNTIME DESTINATION ${INSTALL_BINDIR}
    LIBRARY DESTINATION ${INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${INSTALL_ARCHIVEDIR}
)
