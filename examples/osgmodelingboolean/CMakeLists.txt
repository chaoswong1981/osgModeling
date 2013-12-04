SET(EXAMPLE_NAME osgmodelingboolean)
SET(EXAMPLE_FILES
    osgmodelingboolean.cpp
)
ADD_EXECUTABLE(${EXAMPLE_NAME} ${EXAMPLE_FILES})
SET_TARGET_PROPERTIES(${EXAMPLE_NAME} PROPERTIES PROJECT_LABEL "${EXAMPLE_NAME}")
SET_TARGET_PROPERTIES(${EXAMPLE_NAME} PROPERTIES DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
SET_TARGET_PROPERTIES(${EXAMPLE_NAME} PROPERTIES OUTPUT_NAME ${EXAMPLE_NAME})

TARGET_LINK_LIBRARIES(${EXAMPLE_NAME}
    debug osg${OSG_DEBUG_POSTFIX}         optimized osg
    debug osgUtil${OSG_DEBUG_POSTFIX}     optimized osgUtil
    debug osgViewer${OSG_DEBUG_POSTFIX}   optimized osgViewer
    debug osgText${OSG_DEBUG_POSTFIX}     optimized osgText
    debug osgDB${OSG_DEBUG_POSTFIX}       optimized osgDB
    debug osgGA${OSG_DEBUG_POSTFIX}       optimized osgGA
    debug OpenThreads${OSG_DEBUG_POSTFIX} optimized OpenThreads
    debug osgModeling${OSG_DEBUG_POSTFIX} optimized osgModeling
)
INSTALL(TARGETS ${EXAMPLE_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
