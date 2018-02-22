#pragma once
#include "tools\CToolCursor.h"
#include "tools/CGenericTool.h"
#include "queue"

using namespace std;
using namespace chai3d;

class cToolDelayCursor :
	public cGenericTool
{
public:
	cToolDelayCursor(cWorld* a_parentWorld);
	~cToolDelayCursor();


	//double* m_forceArray[MAX_DELAY_FRAME];

	//unsigned m_forceArrayIndex;

	unsigned int m_delayFrame;
	bool m_trimed;
	queue <cVector3d> m_forceQueue;

	// Single haptic point representing a cursor.
	cHapticPoint* m_hapticPoint;
	
	//! This method computes the interaction forces between the haptic point and the environment.
	virtual void computeInteractionForces();

	//! This method renders the tools using OpenGL.
	virtual void render(cRenderOptions& a_options);

	//! This method updates the position and orientation of the tool image.
	virtual void updateToolImagePosition();

	void addForceToQuene(cVector3d force);
	cVector3d getForceFromQuene(unsigned int nDelay);
	void updateDelayFrame(unsigned int ndelayFrame);
	cGenericObject* m_delayObject;
};

