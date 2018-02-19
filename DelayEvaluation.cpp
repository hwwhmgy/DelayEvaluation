//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1925 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include "cToolDelayCursor.h"
#include "COculus.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;
//result file
FILE* fileCache = NULL;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolDelayCursor* tool;

// a few objects that are placed in the scene
cMultiSegment* segments;

// a colored background
cBackground* background;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

cLabel* labelTestLoop;
//participant index
int participantindex = 0;
//num of test for each participant
int maxtest = 100;
//cuttent test index
int curLoop = 0;
//participant give an answer
bool bAnswered = false;
//stiffness coefficient for the three object, now all of the three are given the same value
float stiffcoeff1 = 0.5;
float stiffcoeff2 = 0.5;
float stiffcoeff3 = 0.5;
//the index of the object which the delay is applied
int DelayObjIndex = 0;
//current delay time
double timeDelay = 0.002;
//max delay time
double maxTimeDelay = 0.01;
//max stiffness
double maxStiffness;
//delay frames
unsigned int nDelay = 0;
// pointer of the three base objects
cMesh* base[3] = { NULL,NULL,NULL };

//point of the three pipes
cMesh*  cylinder[3] = { NULL,NULL,NULL };

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

void updateTestLoop(void);
//------------------------------------------------------------------------------
// OCULUS RIFT
//------------------------------------------------------------------------------

// display context
cOVRRenderContext renderContext;

// oculus device
cOVRDevice oculusVR;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);




//==============================================================================

int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "Delay in haptci rendering" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Randomly delay will be added to the haptic rendering loop in contact with one of the three objects" << endl << endl;
	cout << "Use the delta to touch the three objects and choose the one with delay:" << endl;
	cout << "[1] - the left one" << endl;
	cout << "[2] - the middle one" << endl;
	cout << "[3] - the right one" << endl;
	cout << endl << endl;


	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);


	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif

	// initialize oculus
	if (!oculusVR.initVR())
	{
		cout << "failed to initialize Oculus" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get oculus display resolution
	ovrSizei hmdResolution = oculusVR.getResolution();

	// setup mirror display on computer screen
	ovrSizei windowSize = { hmdResolution.w / 2, hmdResolution.h / 2 };

	// inialize buffers
	if (!oculusVR.initVRBuffers(windowSize.w, windowSize.h))
	{
		cout << "failed to initialize Oculus buffers" << endl;
		cSleepMs(1000);
		oculusVR.destroyVR();
		renderContext.destroy();
		glfwTerminate();
		return 1;
	}
	// set window size
	glfwSetWindowSize(window, windowSize.w, windowSize.h);
	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setWhite();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.9, 0.0, 0.6),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

// set the near and far clipping planes of the camera
// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.03);
	camera->setStereoFocalLength(1.8);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(0.6, 0.6, 0.5);

	// define the direction of the light beam
	light->setDir(-0.5, -0.5, -0.5);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(false);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityMedium();

	// set light cone half angle
	light->setCutOffAngleDeg(30);


	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get access to the first available haptic device
	handler->getDevice(hapticDevice, 0);

	// retrieve information about the current haptic device
	cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

	// if the haptic devices carries a gripper, enable it to behave like a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	// create a tool (cursor) and insert into the world
	tool = new cToolDelayCursor(world);
	world->addChild(tool);

	// connect the haptic device to the tool
	tool->setHapticDevice(hapticDevice);

	// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setWorkspaceRadius(1.0);

	// define the radius of the tool (sphere)
	double toolRadius = 0.05;

	// define a radius for the tool
	tool->setRadius(toolRadius);

	// hide the device sphere. only show proxy.
	tool->setShowContactPoints(true, false);

	// enable if objects in the scene are going to rotate of translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);

	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// start the haptic tool
	tool->start();


	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

	// stiffness properties
	maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;


	/////////////////////////////////////////////////////////////////////////
	// BASE
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	base[0] = new cMesh();
	//base = base1;
	base[0]->m_name = "base1";

	// add object to world
	world->addChild(base[0]);

	// build mesh using a cylinder primitive
	cCreateCylinder(base[0],
		0.01,
		0.2,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, -0.4, -0.01),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	//base1->m_material->setGrayGainsboro();
	base[0]->m_material->setRedDarkSalmon();
	base[0]->m_material->setStiffness(0.3 * maxStiffness);

	// build collision detection tree
	base[0]->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base[0]->setUseDisplayList(true);
	//base[0]->setEnabled(false);

	//////////////////////////////////////////////////////////////////////////
	// create a mesh
	base[1] = new cMesh();
	base[1]->m_name = "base2";
	// add object to world
	world->addChild(base[1]);

	// build mesh using a cylinder primitive
	cCreateCylinder(base[1],
		0.01,
		0.2,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.0, -0.01),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	//base2->m_material->setGrayGainsboro();
	base[1]->m_material->setGreenYellowGreen();
	base[1]->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	base[1]->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base[1]->setUseDisplayList(true);
	//base[1]->setEnabled(false);
	////////////////////////////////////////////////////////////////////////
	// create a mesh
	base[2] = new cMesh();
	base[2]->m_name = "base3";
	// add object to world
	world->addChild(base[2]);

	// build mesh using a cylinder primitive
	cCreateCylinder(base[2],
		0.01,
		0.2,
		36,
		1,
		10,
		true,
		true,
		cVector3d(0.0, 0.4, -0.01),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ)
	);

	// set material properties
	//base3->m_material->setGrayGainsboro();
	base[2]->m_material->setBlueDeepSky();
	base[2]->m_material->setStiffness(0.7 * maxStiffness);

	// build collision detection tree
	base[2]->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	base[2]->setUseDisplayList(true);


	
	/////////////////////////////////////////////////////////////////////////
	// pipe
	/////////////////////////////////////////////////////////////////////////

	// create a mesh
	cylinder[0] = new cMesh();

	// add object to world
	world->addChild(cylinder[0]);

	// build mesh using a cylinder primitive
	cCreatePipe(cylinder[0],
		0.15,
		0.05,
		0.06,
		32,
		1,
		cVector3d(0.0, -0.2, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(170), C_EULER_ORDER_XYZ)
	);

	// set material properties
	cylinder[0]->m_material->setRedDarkSalmon();
	cylinder[0]->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	cylinder[0]->createAABBCollisionDetector(toolRadius);
	//cylinder->create

	// use display list to optimize graphic rendering performance
	cylinder[0]->setUseDisplayList(true);

	///////////////////////////////////////////////////////////////////////
	// 
	cylinder[1] = new cMesh();

	// add object to world
	world->addChild(cylinder[1]);

	// build mesh using a cylinder primitive
	cCreatePipe(cylinder[1],
		0.15,
		0.05,
		0.06,
		32,
		1,
		cVector3d(0.0, 0.0, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(170), C_EULER_ORDER_XYZ)
	);

	// set material properties
	cylinder[1]->m_material->setGreenYellowGreen();
	cylinder[1]->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	cylinder[1]->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cylinder[1]->setUseDisplayList(true);
	//////////////////////////////////////////////////////////////////
	// create a mesh
	cylinder[2] = new cMesh();

	// add object to world
	world->addChild(cylinder[2]);

	// build mesh using a cylinder primitive
	cCreatePipe(cylinder[2],
		0.15,
		0.05,
		0.06,
		32,
		1,
		cVector3d(0.0, 0.2, 0.0),
		cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(170), C_EULER_ORDER_XYZ)
	);

	// set material properties
	cylinder[2]->m_material->setBlueCornflower();
	cylinder[2]->m_material->setStiffness(0.5 * maxStiffness);

	// build collision detection tree
	cylinder[2]->createAABBCollisionDetector(toolRadius);

	// use display list to optimize graphic rendering performance
	cylinder[2]->setUseDisplayList(true);

	for (int i = 0; i < 3; i++)
	{
		cylinder[i]->setEnabled(false);
	}
	
	fileCache = fopen("./record0.txt", "w");

   
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

	labelTestLoop = new cLabel(font);
	labelTestLoop->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelTestLoop);
	

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.8f, 0.8f, 0.8f),
                                cColorf(0.8f, 0.8f, 0.8f));


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);

	// recenter oculus
	//oculusVR.recenterPose();
    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
		
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

		//
		updateTestLoop();
        // render graphics
        updateGraphics();

		/*// start rendering
		oculusVR.onRenderStart();

		// render frame for each eye
		for (int eyeIndex = 0; eyeIndex < ovrEye_Count; eyeIndex++)
		{
			// retrieve projection and modelview matrix from oculus
			cTransform projectionMatrix, modelViewMatrix;
			oculusVR.onEyeRender(eyeIndex, projectionMatrix, modelViewMatrix);

			camera->m_useCustomProjectionMatrix = true;
			camera->m_projectionMatrix = projectionMatrix;

			camera->m_useCustomModelViewMatrix = true;
			camera->m_modelViewMatrix = modelViewMatrix;

			// render world
			ovrSizei size = oculusVR.getEyeTextureSize(eyeIndex);
			camera->renderView(size.w, size.h, C_STEREO_LEFT_EYE, false);

			// finalize rendering  
			oculusVR.onEyeRenderFinish(eyeIndex);
		}

		// update frames
		oculusVR.submitFrame();
		oculusVR.blitMirror();*/

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
	double frequency = freqCounterHaptics.getFrequency();
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }
	else if ((a_key == GLFW_KEY_1))
	{
		fprintf(fileCache, "%d,%f,%f,%f,%d,1\n", curLoop, stiffcoeff1*maxStiffness, timeDelay, frequency, DelayObjIndex);
		bAnswered = true;
	}
	else if ((a_key == GLFW_KEY_2))
	{
		fprintf(fileCache, "%d,%f,%f,%f,%d,2\n", curLoop, stiffcoeff1*maxStiffness, timeDelay, frequency, DelayObjIndex);
		bAnswered = true;
	}
	else if ((a_key == GLFW_KEY_3))
	{
		fprintf(fileCache, "%d,%f,%f,%f,%d,3\n", curLoop, stiffcoeff1*maxStiffness, timeDelay, frequency, DelayObjIndex);
		bAnswered = true;
	}
	else if ((a_key == GLFW_KEY_4))
	{
		fprintf(fileCache, "%d,%f,%f,%f,%d,4\n", curLoop, stiffcoeff1*maxStiffness, timeDelay, frequency, DelayObjIndex);
		bAnswered = true;
	}
    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }
    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }
	else if (a_key == GLFW_KEY_UP)
	{
		if (timeDelay < 0.05) timeDelay = timeDelay + 0.001;
	}
	else if(a_key == GLFW_KEY_DOWN)
	{
		if (timeDelay >0.002) timeDelay = timeDelay - 0.001;
	}
	
}

//------------------------------------------------------------------------------

void close(void)
{
	fclose(fileCache);
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////
	double frequency = freqCounterHaptics.getFrequency();
	double delay = 0.0;
	if (frequency <1)
	{
		nDelay = 0;
	}
	else
	{
		delay = timeDelay / (1 / frequency);
		nDelay = (unsigned int)delay;
		//tool->updateDelayFrame(nDelay);
	}
	//tool->updateDelayFrame(nDelay);
	printf("stiffness:%f Obj: %d delay:%f  %f  %d   frenquency %f\n", stiffcoeff1*maxStiffness, DelayObjIndex,timeDelay, delay, nDelay, frequency);

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	labelTestLoop->setText("Test"+cStr(participantindex) +":"+ cStr(curLoop)+"/" + cStr(maxtest));
	labelTestLoop->setLocalPos((int)(0.5 * (width - labelTestLoop->getWidth())), 35);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics(void)
{
    cMode state = IDLE;
    cGenericObject* object = NULL;
    cTransform tool_T_object;

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

		double frequency = freqCounterHaptics.getFrequency();
		double delay = 0.0;
		if (frequency <1)
		{
			nDelay = 0;
		}
		else
		{
			delay = timeDelay / (1 / frequency);
			nDelay = (unsigned int)delay;
			//tool->updateDelayFrame(nDelay);
		}
		tool->updateDelayFrame(nDelay);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

 
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // compute transformation from world to tool (haptic device)
        cTransform world_T_tool = tool->getDeviceGlobalTransform();

        // get status of user switch
        bool button = tool->getUserSwitch(0);

        //
        // STATE 1:
        // Idle mode - user presses the user switch
        //
        if ((state == IDLE) && (button == true))
        {
            // check if at least one contact has occurred
            if (tool->m_hapticPoint->getNumCollisionEvents() > 0)
            {
                // get contact event
                cCollisionEvent* collisionEvent = tool->m_hapticPoint->getCollisionEvent(0);

                // get object from contact event
                object = collisionEvent->m_object;

                // get transformation from object
                cTransform world_T_object = object->getGlobalTransform();

                // compute inverse transformation from contact point to object 
                cTransform tool_T_world = world_T_tool;
                tool_T_world.invert();

                // store current transformation tool
                tool_T_object = tool_T_world * world_T_object;

                // update state
                state = SELECTION;
            }
        }

        //
        // STATE 2:
        // Selection mode - operator maintains user switch enabled and moves object
        //
        else if ((state == SELECTION) && (button == true))
        {
            // compute new tranformation of object in global coordinates
            cTransform world_T_object = world_T_tool * tool_T_object;

            // compute new tranformation of object in local coordinates
            cTransform parent_T_world = object->getParent()->getLocalTransform();
            parent_T_world.invert();
            cTransform parent_T_object = parent_T_world * world_T_object;

            // assign new local transformation to object
            object->setLocalTransform(parent_T_object);

            // set zero forces when manipulating objects
            tool->setDeviceGlobalForce(0.0, 0.0, 0.0);
        }

        //
        // STATE 3:
        // Finalize Selection mode - operator releases user switch.
        //
        else
        {
            state = IDLE;
        }


        /////////////////////////////////////////////////////////////////////////
        // FINALIZE
        /////////////////////////////////////////////////////////////////////////

        // send forces to haptic device
		
        tool->applyToDevice();  
    }
    
    // exit haptics thread
    simulationFinished = true;
}
void updateTestLoop(void)
{
	if (bAnswered || curLoop == 0)
	{
		srand(time(NULL));
		//int m = rand()%100;
		//int n = rand()%100 / 100;
		stiffcoeff1 = (float)(rand() % 95) / 100 +0.05;
		stiffcoeff2 = stiffcoeff1;// (float)(rand() % 100) / 100;
		stiffcoeff3 = stiffcoeff1;// (float)(rand() % 100) / 100;

		base[0]->setStiffness(stiffcoeff1*maxStiffness);
		base[1]->setStiffness(stiffcoeff2*maxStiffness);
		base[2]->setStiffness(stiffcoeff3*maxStiffness);

		cylinder[0]->setStiffness(stiffcoeff1*maxStiffness);
		cylinder[1]->setStiffness(stiffcoeff2*maxStiffness);
		cylinder[2]->setStiffness(stiffcoeff3*maxStiffness);

		DelayObjIndex = rand() % 3;
		tool->m_delayObject = base[DelayObjIndex];
		float delayCoeff = (float)(rand() % 100) / 100;
		timeDelay = delayCoeff * maxTimeDelay;
		
		if(curLoop < maxtest)
		{
			curLoop++;
			bAnswered = false; 
		}
		else
		{
			curLoop = 0;
			participantindex++;
			char filename[256] = "./record";
			char index[256];
			sprintf(index, "%d.txt", participantindex);
			strcat(filename, index);
			if (fileCache != NULL)
			{
				fclose(fileCache);
				fileCache = NULL;
			}
			fileCache = fopen(filename, "w");
			//printf("test finished\n");fileCache = fopen( "./record0.txt", "w");
		}
	}
}
//------------------------------------------------------------------------------
