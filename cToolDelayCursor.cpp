#include "cToolDelayCursor.h"
#include "tools/CToolCursor.h"
#include "graphics/CTriangleArray.h"

using namespace chai3d;


cToolDelayCursor::cToolDelayCursor(cWorld* a_parentWorld) :cGenericTool(a_parentWorld)
, m_delayObject(NULL)
{
	// create a single point contact
	m_hapticPoint = new cHapticPoint(this);

	// add point to list
	m_hapticPoints.push_back(m_hapticPoint);

	// show proxy spheres only
	setShowContactPoints(true, false);

	m_trimed = true;
}


cToolDelayCursor::~cToolDelayCursor()
{
	delete m_hapticPoint;
}

/*
This method updates the position and orientation of the tool image.
*/
//==============================================================================
void cToolDelayCursor::updateToolImagePosition()
{
	// set the position and orientation of the tool image to be equal to the 
	// one of the haptic point proxy.
	cVector3d pos = m_hapticPoint->getLocalPosProxy();
	m_image->setLocalPos(pos);
	m_image->setLocalRot(m_deviceLocalRot);
	m_forceQueue.empty();
}


//==============================================================================
/*!
This method computes the interaction forces between the haptic point and
the virtual environment.
*/
//==============================================================================
void cToolDelayCursor::computeInteractionForces()
{
	// compute interaction forces at haptic point in global coordinates
	cVector3d globalForce = m_hapticPoint->computeInteractionForces(m_deviceGlobalPos,
		m_deviceGlobalRot,
		m_deviceGlobalLinVel,
		m_deviceGlobalAngVel);
	//m_hapticPoint->
	//m_hapticPoint->isInContact();
	/*int num = m_hapticPoint->getNumCollisionEvents();
	cCollisionEvent * cColEvent = NULL;
	for (int i = 0; i < num; i++)
	{
		cColEvent = m_hapticPoint->getCollisionEvent(i);
		if(cColEvent != NULL)
			cColEvent->
	}*/
	if(m_hapticPoint->isInContact(m_delayObject))
	{
		addForceToQuene(globalForce);
		globalForce = getForceFromQuene(m_delayFrame);
	}	


	cVector3d globalTorque(0.0, 0.0, 0.0);

	// update computed forces to tool
	setDeviceGlobalForce(globalForce);
	setDeviceGlobalTorque(globalTorque);
	setGripperForce(0.0);
}


//==============================================================================
/*!
This method renders the current tool using OpenGL.

\param  a_options  Rendering options.
*/
//==============================================================================
void cToolDelayCursor::render(cRenderOptions& a_options)
{
	///////////////////////////////////////////////////////////////////////
	// render haptic points
	///////////////////////////////////////////////////////////////////////
	int numContactPoint = (int)(m_hapticPoints.size());
	for (int i = 0; i<numContactPoint; i++)
	{
		// get next haptic point
		cHapticPoint* nextContactPoint = m_hapticPoints[i];

		// render tool
		nextContactPoint->render(a_options);
	}

	///////////////////////////////////////////////////////////////////////
	// render mesh image
	///////////////////////////////////////////////////////////////////////
	if (m_image != NULL)
	{
		m_image->renderSceneGraph(a_options);
	}
}




void cToolDelayCursor::addForceToQuene(cVector3d force)
{
	if(m_forceQueue.size()>1000)
	{ 
		m_forceQueue.pop();
		m_forceQueue.push(force);
	}
	else
	{
		m_forceQueue.push(force);
	}
}


cVector3d cToolDelayCursor::getForceFromQuene(unsigned int nDelay)
{
	if (m_forceQueue.size() == 0)
	{
		return cVector3d(0.0, 0.0, 0.0);
	}
	if (m_trimed)
	{
		if (nDelay > m_forceQueue.size())
		{
			return cVector3d(0.0, 0.0, 0.0);
		}
		else
		{
			while (nDelay < m_forceQueue.size() - 1)
			{
				m_forceQueue.pop();
			}
			if (m_forceQueue.size() > 0)
			{
				cVector3d  vec = m_forceQueue.front();
				m_forceQueue.pop();
				return vec;
			}
			else
			{
				return cVector3d(0.0, 0.0, 0.0);
			}
		}
	}
	else
	{
		if (nDelay > m_forceQueue.size())
		{
			return cVector3d(0.0, 0.0, 0.0);
		}
		else
		{
			cVector3d  vec = m_forceQueue.front();
			m_forceQueue.pop();
			return vec;
		}
	}
	
	
}


void cToolDelayCursor::updateDelayFrame(unsigned int ndelayFrame)
{
	m_delayFrame = ndelayFrame;
}
