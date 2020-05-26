#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <osgGA/CameraManipulator>
#include <osg/PositionAttitudeTransform>
#include <osg/Transform>

#include <osgText/Text>
#include <osgText/Font>
// class to allow access to matrix that represents accumlation of 
//  matrices above specified node of scene graph.
struct updateAccumulatedMatrix : public osg::NodeCallback
{
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		matrix = osg::computeWorldToLocal(nv->getNodePath());
		traverse(node, nv);
	}
	osg::Matrix matrix;
};

struct transformAccumulator
{
	transformAccumulator();
	bool attachToGroup(osg::Group* g);
	osg::Matrix getMatrix();
protected:
	osg::ref_ptr<osg::Group> parent;
	osg::Node* node;
	updateAccumulatedMatrix* mpcb;
};
