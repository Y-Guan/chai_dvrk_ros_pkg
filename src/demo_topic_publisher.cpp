#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
#include "chai/demo_msg.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/transform_listener.h"
#include <iostream>
#include <chai3d.h>
#include <stdlib.h>
#include <stdio.h>
#include "GLFW/glfw3.h"
//#include "../external/chai3d-3.2.0/extras/GLFW/include/GLFW/glfw3.h"
//#include "../glfw-master/include/GLFW/glfw3.h"

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// You may need to define the objects here.
// ...
// a few spherical objects
cShapeSphere* object0;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a colored background
cBackground* background;

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

// a first window
GLFWwindow* window1 = NULL;
int width1 = 0;
int height1 = 0;

// a second window
GLFWwindow* window2 = NULL;
int width2 = 0;
int height2 = 0;

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

// callback when tf
void tf_position_cb(const geometry_msgs::Vector3::ConstPtr & msg);

// callback when the window1 display is resized
void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window2 display is resized
void windowSizeCallback2(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene1
void updateGraphics1(void);

// this function renders the scene2
void updateGraphics2(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


//--------------------------------------------------------------------------
// MAIN
//--------------------------------------------------------------------------

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
    cout << "Yuan Guan's Template" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);
    std::cout << resourceRoot  << endl;

    //--------------------------------------------------------------------------
    // Init ROS Node
    //--------------------------------------------------------------------------

     // init a node with name "demo_topic_publisher_node"
     ros::init(argc, argv, "demo_topic_publisher_node");
     // create a object for the node to communicate with ROS system
     ros::NodeHandle node_obj;

     /* ######################################################
     Create a topic publisher:
         - topic name: /dvrk/MTMR/state_joint_current
         - message type: std::msgs::Int32
         - queue size: 10 (set to high if sending rate is high)
     #######################################################*/
     ros::Publisher jointState_publisher = node_obj.advertise<sensor_msgs::JointState>("/dvrk/MTMR/state_joint_current",10);

     // set the frequency of sending data
     ros::Rate loop_rate(30);

     // create message3
     sensor_msgs::JointState msg;
     msg.position.resize(8);
     msg.name.resize(8);
     msg.name[0] = "outer_yaw";
     msg.name[1] = "shoulder_pitch";
     msg.name[2] = "shoulder_pitch_parallel";
     msg.name[3] = "elbow_pitch";
     msg.name[4] = "wrist_platform";
     msg.name[5] = "wrist_pitch";
     msg.name[6] = "wrist_yaw";
     msg.name[7] = "wrist_roll";
     msg.position[0] = 0.0;
     msg.position[1] = 0.0;
     msg.position[2] = 0.0;
     msg.position[3] = 0.0;
     msg.position[4] = 0.0;
     msg.position[5] = 0.0;
     msg.position[6] = 0.0;
     msg.position[7] = 0.0;
     double increment0 = 0.01;
     double increment1 = 0.01;
     double increment2 = 0.01;
     double increment3 = 0.01;

     tf2_ros::Buffer tfBuffer;
     tf2_ros::TransformListener tfListener(tfBuffer);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        std::cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int space = 10;
    //int w = 0.5 * mode->height;
    //int h = 0.5 * mode->height;
    int w = 640;
    int h = 480;
    int x1 = 0.5 * mode->width - w - space;
    int y1 = 0.5 * (mode->height - h);
    int x2 = 0.5 * mode->width + space;
    int y2 = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 1
    ////////////////////////////////////////////////////////////////////////////

    // create display context
    window1 = glfwCreateWindow(w, h, "Left eye", NULL, NULL);
    if (!window1)
    {
        std::cout << "failed to create window1" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window1, &width1, &height1);

    // set position of window
    glfwSetWindowPos(window1, x1, y1);

    // set key callback
    glfwSetKeyCallback(window1, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window1, windowSizeCallback1);

    // set current display context
    glfwMakeContextCurrent(window1);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);


    ////////////////////////////////////////////////////////////////////////////
    // SETUP WINDOW 2
    ////////////////////////////////////////////////////////////////////////////

    // create display context and share GPU data with window 0
    window2 = glfwCreateWindow(w, h, "Right eye", NULL, window1);
    if (!window2)
    {
        std::cout << "failed to create window2" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window2, &width2, &height2);

    // set position of window
    glfwSetWindowPos(window2, x2, y2);

    // set key callback
    glfwSetKeyCallback(window2, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window2, windowSizeCallback2);

    // set current display context
    glfwMakeContextCurrent(window2);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    ////////////////////////////////////////////////////////////////////////////
    // GLEW
    ////////////////////////////////////////////////////////////////////////////

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        std::cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif

    //--------------------------------------------------------------------------
    // WORLD
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setWhite();

    //--------------------------------------------------------------------------
    // CAMERA
    //--------------------------------------------------------------------------

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.0, -3.0, 1.0),    // camera position (eye)
                cVector3d(0.5, 0.0, 1.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 100);

    // set stereo mode
    camera->setStereoMode(C_STEREO_PASSIVE_DUAL_DISPLAY);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    //camera->setStereoEyeSeparation(0.03);
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    //--------------------------------------------------------------------------
    // LIGHTING
    //--------------------------------------------------------------------------

    // create a light source
    light = new cDirectionalLight(world);

    // add light to world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-1.0,-1.0, -1.0);

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

    // define the radius of the tool (sphere)
    double toolRadius = 0.03;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    //tool->setShowContactPoints(false, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setBlack();

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

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

    // properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    /////////////////////////////////////////////////////////////////////////
    // OTHER OBJECTS
    /////////////////////////////////////////////////////////////////////////
    // ...
        /////////////////////////////////////////////////////////////////////////
        // OBJECT 0: "MAGNET"
        /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object0 = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object0);

    // set the position of the object at the center of the world
    object0->setLocalPos(0.0, 0.0, 0.0);

    // load texture map
    bool fileload;
//    object0->m_texture = cTexture2d::create();
//    //fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg"));
//    fileload = object0->m_texture->loadFromFile(RESOURCE_PATH("../../../src/chai/external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg"));
//    //fileload = object0->m_texture->loadFromFile("../external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg");
//    //std::cout << RESOURCE_PATH << endl;
//    if (!fileload)
//    {
//        std::cout << "Error0 - Texture image failed to load correctly." << endl;
//        #if defined(_MSVC)
//        fileload = object0->m_texture->loadFromFile("../../../bin/resources/images/spheremap-3.jpg");
//        //fileload = object0->m_texture->loadFromFile("../external/chai3d-3.2.0/bin/resources/images/spheremap-3.jpg");
//        #endif
//    }
//    if (!fileload)
//    {
//        std::cout << "Error1 - Texture image failed to load correctly." << endl;
//        close();
//        return (-1);
//    }
//
//    // set graphic properties
//    object0->m_texture->setSphericalMappingEnabled(true);
//    //object0->setUseTexture(true);
//    object0->setUseTexture(true);
    object0->m_material->setWhite();

    // get properties of haptic device
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

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

    cout << "here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setBlack();
    camera->m_frontLayer->addChild(labelRates);

    // create a background
    background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(1.0f, 1.0f, 1.0f),
                                cColorf(0.9f, 0.9f, 0.9f),
                                cColorf(0.9f, 0.9f, 0.9f));


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
    windowSizeCallback1(window1, width1, height1);
    windowSizeCallback2(window2, width2, height2);

    cout << "here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;

    // main graphic loop
    //while ((!glfwWindowShouldClose(window1)) && (!glfwWindowShouldClose(window2)) && ros::ok())
    while ((!glfwWindowShouldClose(window1)) && (!glfwWindowShouldClose(window2)) )
    {
        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 1
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window1);

        // get width and height of window
        glfwGetWindowSize(window1, &width1, &height1);

        // render graphics
        updateGraphics1();

        // swap buffers
        glfwSwapBuffers(window1);


        ////////////////////////////////////////////////////////////////////////
        // RENDER WINDOW 2
        ////////////////////////////////////////////////////////////////////////

        // activate display context
        glfwMakeContextCurrent(window2);

        // get width and height of window
        glfwGetWindowSize(window2, &width2, &height2);

        // render graphics
        updateGraphics2();

        // swap buffers
        glfwSwapBuffers(window2);


        ////////////////////////////////////////////////////////////////////////
        // FINALIZE
        ////////////////////////////////////////////////////////////////////////

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);


        msg.position[0] = msg.position[0] + increment0;
        msg.position[1] = msg.position[1] + increment1;
        if (msg.position[0] > 0.15 || msg.position[0] < -0.15) {
            increment0 = -increment0;
        }
        if (msg.position[1] > 0.2 || msg.position[1] < -0.2) {
            increment1 = -increment1;
        }
        if (msg.position[2] > 0.1 || msg.position[2] < -0.1) {
            increment1 = -increment1;
        }

        //publish
        jointState_publisher.publish(msg);

        //ros::Subscriber tf_position_sub = node_obj.subscribe<geometry_msgs::Vector3>("/tf/transforms[7]/transform/translation", 10, tf_position_cb);
        geometry_msgs::TransformStamped T;

        try
        {
            //T = tfBuffer.lookupTransform("world", "link6", ros::Time(0));
            T = tfBuffer.lookupTransform("world", "MTMR_wrist_roll_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        std::cout << T << std::endl;
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(0) = T.transform.translation.x;
        object0_pos(1) = T.transform.translation.y;
        object0_pos(2) = T.transform.translation.z;
        cout << object0_pos << endl;
        double scale_pos = 1.5;
        object0->setLocalPos(scale_pos*object0_pos(0),scale_pos*object0_pos(1),scale_pos*object0_pos(2));

        //read and update all the topics
        ros::spinOnce();
        //delay to achieve desired publishing rate
        loop_rate.sleep();


    }

    // close windows
    glfwDestroyWindow(window1);
    glfwDestroyWindow(window2);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------
// object movement functions
//------------------------------------------------------------------------------
void tf_position_cb(const geometry_msgs::Vector3::ConstPtr & msg)
{
    cVector3d object0_pos = object0->getLocalPos();
    object0_pos(0) = msg->z;
    object0_pos(1) = msg->x;
    object0_pos(2) = msg->y;
    cout << object0_pos << endl;
    object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));

}

//------------------------------------------------------------------------------
// CALLBACK WHEN CHANGING WINDOW SIZE
//------------------------------------------------------------------------------
void windowSizeCallback1(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width1  = a_width;
    height1 = a_height;
}
//------------------------------------------------------------------------------
void windowSizeCallback2(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width2  = a_width;
    height2 = a_height;
}

//------------------------------------------------------------------------------
// ERROR CALLBACK
//------------------------------------------------------------------------------
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;
    cout << "Error: " << a_description << endl;
    cout << "here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl << endl;
}

//------------------------------------------------------------------------------
// KEY CALLBACK
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

    // option - up
    else if (a_key == GLFW_KEY_UP)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(2) = object0_pos(2) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }

    // option - down
    else if (a_key == GLFW_KEY_DOWN)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(2) = object0_pos(2) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }

    // option - left
    else if (a_key == GLFW_KEY_LEFT)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(1) = object0_pos(1) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }

    // option - right
    else if (a_key == GLFW_KEY_RIGHT)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(1) = object0_pos(1) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }

    // option - away
    else if (a_key == GLFW_KEY_W)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(0) = object0_pos(0) - 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }

    // option - close
    else if (a_key == GLFW_KEY_S)
    {
        cVector3d object0_pos = object0->getLocalPos();
        object0_pos(0) = object0_pos(0) + 0.01;
        cout << object0_pos << endl;
        object0->setLocalPos(object0_pos(0),object0_pos(1),object0_pos(2));
    }
}

//------------------------------------------------------------------------------
// UPDATE GRAPHICS
//------------------------------------------------------------------------------
void updateGraphics1(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////
    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");
    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width1 - labelRates->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE 1
    /////////////////////////////////////////////////////////////////////
    // update shadow maps (if any)
    world->updateShadowMaps(false, false);
    // render world
    camera->renderView(width1, height1, C_STEREO_LEFT_EYE);
    // wait until all GL commands are completed
    glFinish();
    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------
void updateGraphics2(void)
{
    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE 2
    /////////////////////////////////////////////////////////////////////
    // update shadow maps (if any)
    world->updateShadowMaps(false, false);
    // render world
    camera->renderView(width2, height2, C_STEREO_RIGHT_EYE);
    // wait until all GL commands are completed
    glFinish();
    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}


//------------------------------------------------------------------------------
// UPDATE HAPTICS
//------------------------------------------------------------------------------
void updateHaptics(void)
{
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

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
// CLOSE FUNCTION
//------------------------------------------------------------------------------
void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete [] (hapticsThread);
    delete [] (world);
    delete [] (handler);
//    delete [] camera;
//    delete [] light;
//    delete [] object0;
//    delete [] tool;
//    delete [] background;
//    delete [](labelRates);
//    delete [](window1);
//    delete [](window2);
}

