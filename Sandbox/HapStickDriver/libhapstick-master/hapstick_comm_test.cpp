//==============================================================================
/*
Colette Knopp
Hapstick Communication Test between Arduino and Windows
*/
//==============================================================================
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------

#include <GLFW/glfw3.h>
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
/*
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;
*/

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------
/*
// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing the haptic device
cShapeSphere* cursor;

// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
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
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;
*/

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
/*
// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);
*/
// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

//==============================================================================
/*
 DEMO:   Serial Communication
 
    This is a test file to work on the serial communication between the arduino
    and the computer.
 
    Computer will open a connection with the device, the arduino will send 
    random position information, and the computer will respond with arbitrary
    force information for the arduino. The arduino will send the information 
    back to the device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "HapStick" << endl;
    cout << "Demo: Serial Communication" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();
/*
    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // display a reference frame if haptic device supports orientations
    if (info.m_sensedRotation == true)
    {
        // display reference frame
        cursor->setShowFrame(true);

        // set the size of the reference frame
        cursor->setFrameSize(0.05);
    }

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);
*/

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------
/*
    // create a font
    font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic device model
    labelHapticDeviceModel = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDeviceModel);
    labelHapticDeviceModel->setText(info.m_modelName);

    // create a label to display the position of haptic device
    labelHapticDevicePosition = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticDevicePosition);
    
    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
*/
    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);
/*

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

        // render graphics
        updateGraphics();

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
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;

    // update position of label
    labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

    // update position of label
    labelHapticDevicePosition->setLocalPos(20, height - 60, 0);
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

    // option - enable/disable force field
    else if (a_key == GLFW_KEY_1)
    {
        useForceField = !useForceField;
        if (useForceField)
            cout << "> Enable force field     \r";
        else
            cout << "> Disable force field    \r";
    }

    // option - enable/disable damping
    else if (a_key == GLFW_KEY_2)
    {
        useDamping = !useDamping;
        if (useDamping)
            cout << "> Enable damping         \r";
        else
            cout << "> Disable damping        \r";
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

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}
*/
//------------------------------------------------------------------------------

void close(void)
{
/*    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }
*/
    // close haptic device
    hapticDevice->close();
/*
    // delete resources
    delete hapticsThread;
    delete world;
 */
    delete handler;
}

//------------------------------------------------------------------------------
/*
void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update position data
    labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------
*/
void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read position 
        cVector3d position;
        hapticDevice->getPosition(position);
        /*
        // read orientation 
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // read gripper position
        double gripperAngle;
        hapticDevice->getGripperAngleRad(gripperAngle);

        // read linear velocity 
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        // read angular velocity
        cVector3d angularVelocity;
        hapticDevice->getAngularVelocity(angularVelocity);

        // read gripper angular velocity
        double gripperAngularVelocity;
        hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

        // read user-switch status (button 0)
        bool button0, button1, button2, button3;
        button0 = false;
        button1 = false;
        button2 = false;
        button3 = false;

        hapticDevice->getUserSwitch(0, button0);
        hapticDevice->getUserSwitch(1, button1);
        hapticDevice->getUserSwitch(2, button2);
        hapticDevice->getUserSwitch(3, button3);


        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
       
        // update arrow
        velocity->m_pointA = position;
        velocity->m_pointB = cAdd(position, linearVelocity);

        // update position and orientation of cursor
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        // adjust the  color of the cursor according to the status of
        // the user-switch (ON = TRUE / OFF = FALSE)
        if (button0)
        {
            cursor->m_material->setGreenMediumAquamarine(); 
        }
        else if (button1)
        {
            cursor->m_material->setYellowGold();
        }
        else if (button2)
        {
            cursor->m_material->setOrangeCoral();
        }
        else if (button3)
        {
            cursor->m_material->setPurpleLavender();
        }
        else
        {
            cursor->m_material->setBlueRoyal();
        }

        // update global variable for graphic display update
        hapticDevicePosition = position;

*/
        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // desired position
        cVector3d desiredPosition;
        //desiredPosition.set((position->x) - 1.0, (position->y) - 1.0, (position->z) - 1.0);
/*
        // desired orientation
        cMatrix3d desiredRotation;
        desiredRotation.identity();
*/
        // variables for forces
        cVector3d force (0,0,0);
        cVector3d torque (0,0,0);
        double gripperForce = 0.0;
        //---------------
        //ADDED FOR TEST
        //cVector3d forceField = Kp * (desiredPosition - position);
        cVector3d forceField (-1.0, -1.0, -1.0);
        force.add(forceField);
        //--------------
/*
        // apply force field
        if (useForceField)
        {
            // compute linear force
            double Kp = 25; // [N/m]
            cVector3d forceField = Kp * (desiredPosition - position);
            force.add(forceField);

            // compute angular torque
            double Kr = 0.05; // [N/m.rad]
            cVector3d axis;
            double angle;
            cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
            deltaRotation.toAxisAngle(axis, angle);
            torque = rotation * ((Kr * angle) * axis);
        }
    
        // apply damping term
        if (useDamping)
        {
            cHapticDeviceInfo info = hapticDevice->getSpecifications();

            // compute linear damping force
            double Kv = 1.0 * info.m_maxLinearDamping;
            cVector3d forceDamping = -Kv * linearVelocity;
            force.add(forceDamping);

            // compute angular damping force
            double Kvr = 1.0 * info.m_maxAngularDamping;
            cVector3d torqueDamping = -Kvr * angularVelocity;
            torque.add(torqueDamping);

            // compute gripper angular damping force
            double Kvg = 1.0 * info.m_maxGripperAngularDamping;
            gripperForce = gripperForce - Kvg * gripperAngularVelocity;
        }
 */

        // send computed force, torque, and gripper force to haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
