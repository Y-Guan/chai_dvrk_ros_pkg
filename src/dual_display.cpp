//------------------------------------------------------------------------------
#include "chai3d.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
#include "chai/demo_msg.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include <iostream>
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <stdio.h>
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
//cStereoMode stereoMode = C_STEREO_DISABLED;
cStereoMode stereoMode = C_STEREO_PASSIVE_DUAL_DISPLAY;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few spherical objects
cShapeSphere* object0;
cShapeSphere* object1;
cShapeSphere* object2;
cShapeSphere* object3;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

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
GLFWwindow* window_R = NULL;
GLFWwindow* window_L = NULL;

// a colored background
cBackground* background;


// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;


//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


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
void updateGraphics_L(void);
void updateGraphics_R(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:   11-effects.cpp

    This example illustrates the use of haptic effects. The application
    begins by creating four spheres with different graphical (
    material and texture) properties. For each sphere, one or more effects
    are programmed to obtain the desired haptic illusion. Parameters to
    adjust each effect are located in the cMaterial class.

    If you wish to develop your own shapes and force effect, take a look at
    the example located in "src/effects". You can also implement your own
    shapes by following the approach presented in the cShapeSphere,
    cShapeTorus or cShapeLine classes for instance.

    In the main haptics loop function  "updateHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{

    printf("argc = %d\n", argc);
    printf("argv = %s\n", argv[0]);

    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 11-effects" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);

    cout << resourceRoot << endl;

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
//        glfwWindowHint(GLFW_STEREO, GL_TRUE);
        glfwWindowHint(GLFW_STEREO, GL_FALSE);

    }

    // create display context
    window_L = glfwCreateWindow(w, h, "CHAI3D_L", NULL, NULL);
    window_R = glfwCreateWindow(w, h, "CHAI3D_R", NULL, window_L);
    if (!window_L)
    {
        cout << "failed to create window L" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window_L, &width, &height);

    // set position of window
    glfwSetWindowPos(window_L, x-432, y);

    // set key callback
    glfwSetKeyCallback(window_L, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window_L, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window_L);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);


    if (!window_R)
    {
        cout << "failed to create window L" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window_R, &width, &height);

    // set position of window
    glfwSetWindowPos(window_R, x-432, y);

    // set key callback
    glfwSetKeyCallback(window_R, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window_R, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window_R);

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


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    //world->m_backgroundColor.setBlack();
    world->m_backgroundColor.set(0.5,0.5,0.5);

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    //camera->setStereoEyeSeparation(0.03);
    camera->setStereoEyeSeparation(0.1);
    camera->setStereoFocalLength(3.0);

//    // set vertical mirrored display mode
//    camera->setMirrorVertical(mirroredDisplay);
//
//    // enable multi-pass rendering to handle transparent objects
//    camera->setUseMultipassTransparency(true);

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0, -1.0, -1.0);
    light->setLocalPos(1.0, 1.0, 1.0);

    // set lighting conditions
    light->m_ambient.set(0.4f, 0.4f, 0.4f);
    light->m_diffuse.set(0.8f, 0.8f, 0.8f);
    light->m_specular.set(1.0f, 1.0f, 1.0f);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.03);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATING OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 0: "MAGNET"
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object0 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object0);

    // set the position of the object at the center of the world
    object0->setLocalPos(0.0, -0.5, 0.0);

    // load texture map
    bool fileload;
    object0->m_texture = cTexture2d::create();
    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = object0->m_texture->loadFromFile("../../../bin/resources/images/spheremap-3.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set graphic properties
    object0->m_texture->setSphericalMappingEnabled(true);
    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // set haptic properties
    object0->m_material->setStiffness(0.4 * maxStiffness);          // % of maximum linear stiffness
    object0->m_material->setMagnetMaxForce(0.6 * maxLinearForce);   // % of maximum linear force
    object0->m_material->setMagnetMaxDistance(0.15);
    object0->m_material->setViscosity(0.1 * maxDamping);            // % of maximum linear damping

    // create a haptic surface effect
    object0->createEffectSurface();

    // create a haptic magnetic effect
    object0->createEffectMagnetic();

    // create a haptic viscous effect
    object0->createEffectViscosity();


    ////////////////////////////////////////////////////////////////////////
    // OBJECT 1: "FLUID"
    ////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object1 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object1);

    // set the position of the object at the center of the world
    object1->setLocalPos(0.0, 0.5, 0.0);

    // load texture map
    object1->m_texture = cTexture2d::create();
    fileload = object1->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/spheremap-2.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = object1->m_texture->loadFromFile("../../../bin/resources/images/spheremap-2.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set graphic properties
    object1->m_material->m_ambient.set(0.1, 0.1, 0.6, 0.5);
    object1->m_material->m_diffuse.set(0.3, 0.3, 0.9, 0.5);
    object1->m_material->m_specular.set(1.0, 1.0, 1.0, 0.5);
    object1->m_material->setWhite();
    object1->setTransparencyLevel(0.6);
    object1->setUseTexture(true);
    object1->m_texture->setSphericalMappingEnabled(true);

    // set haptic properties
    object1->m_material->setViscosity(0.9 * maxDamping);    // % of maximum linear damping

    // create a haptic viscous effect
    object1->createEffectViscosity();


    /////////////////////////////////////////////////////////////////////////
    // OBJECT 2: "STICK-SLIP"
    /////////////////////////////////////////////////////////////////////////

    // create a sphere by define its radius
    object2 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object2);

    // set the position of the object at the center of the world
    object2->setLocalPos(0.0, 0.0, 0.5);

    // load texture map
    object2->m_texture = cTexture2d::create();
    fileload = object2->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/spheremap-5.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = object2->m_texture->loadFromFile("../../../bin/resources/images/spheremap-5.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set graphic properties
    object2->m_texture->setSphericalMappingEnabled(true);
    object2->m_material->setWhite();
    object2->setUseTexture(true);

    // set haptic properties
    object2->m_material->setStickSlipForceMax(0.3 * maxLinearForce);// % of maximum linear force
    object2->m_material->setStickSlipStiffness(0.7 * maxStiffness); // % of maximum linear stiffness

    // create a haptic stick-slip effect
    object2->createEffectStickSlip();


    ////////////////////////////////////////////////////////////////////////
    // OBJECT 3: "VIBRATIONS"
    ////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object3 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object3);

    // set the position of the object at the center of the world
    object3->setLocalPos(0.0, 0.0, -0.5);

    // load texture map
    object3->m_texture = cTexture2d::create();
    fileload = object3->m_texture->loadFromFile(RESOURCE_PATH("../../../../external/chai3d-3.2.0/bin/resources/images/spheremap-4.jpg"));
    if (!fileload)
    {
#if defined(_MSVC)
        fileload = object3->m_texture->loadFromFile("../../../bin/resources/images/spheremap-4.jpg");
#endif
    }
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set graphic properties
    object3->m_texture->setSphericalMappingEnabled(true);
    object3->setUseTexture(true);
    object3->m_material->setWhite();
    object3->setTransparencyLevel(0.9);

    // set haptic properties
    object3->m_material->setVibrationFrequency(50);
    object3->m_material->setVibrationAmplitude(0.1 * maxLinearForce);   // % of maximum linear force
    object3->m_material->setStiffness(0.1 * maxStiffness);              // % of maximum linear stiffness

    // create a haptic vibration effect
    object3->createEffectVibration();

    // create a haptic surface effect
    object3->createEffectSurface();


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.9f, 0.9f, 0.9f),
                                cColorf(0.9f, 0.9f, 0.9f));

    // create a frontground for the endoscope
//    cBackground* background2 = new cBackground();
////    camera2->m_backLayer->addChild(background2);
//
//    // set background properties
//    background2->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
//                                 cColorf(1.0f, 1.0f, 1.0f),
//                                 cColorf(0.9f, 0.9f, 0.9f),
//                                 cColorf(0.9f, 0.9f, 0.9f));


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window_L, width, height);
    windowSizeCallback(window_R, width, height);

    // main graphic loop
    while ((!glfwWindowShouldClose(window_L)) && (!glfwWindowShouldClose(window_R)))
    {
        // activate display context
        glfwMakeContextCurrent(window_L);

        // get width and height of window
        glfwGetWindowSize(window_L, &width, &height);

        // render graphics
        updateGraphics_L();

        // swap buffers
        glfwSwapBuffers(window_L);

//        // process events
//        glfwPollEvents();
//
//        // signal frequency counter
//        freqCounterGraphics.signal(1);


        // activate display context
        glfwMakeContextCurrent(window_R);

        // get width and height of window
        glfwGetWindowSize(window_R, &width, &height);

        // render graphics
        updateGraphics_R();

        // swap buffers
        glfwSwapBuffers(window_R);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);

    }

    // close window
    glfwDestroyWindow(window_L);
    glfwDestroyWindow(window_R);

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
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
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
            glfwSetWindowMonitor(window_L, monitor, 1000, 0, 2*mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window_L, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

        // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
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

void updateGraphics_L(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);
//    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height, C_STEREO_LEFT_EYE);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

void updateGraphics_R(void) {
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int) (0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width, height, C_STEREO_RIGHT_EYE);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
