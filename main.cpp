/*
* Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "imgui.h"


#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "DebugDraw.h"
#include "Test.h"
#include "Car.h"
#include "curve.hpp"

#include "graph.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else

//#include <GL/glew.h>

#endif

#include <GLFW/glfw3.h>

#include <stdio.h>

#define IMGUI_IMPL_OPENGL_LOADER_GLAD

#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>    // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>    // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include "glad/glad.h"  // Initialize with gladLoadGL()
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

//
struct UIState {
    bool showMenu;
    int scroll;
    int scrollarea1;
    bool mouseOverMenu;
    bool chooseTest;
};

//
namespace {
    GLFWwindow *mainWindow = NULL;
    UIState ui;

    int32 testIndex = 0;
    int32 testSelection = 0;
    int32 testCount = 0;
    TestEntry *entry;
    Test *test;
    Settings settings;
    bool rightMouseDown;
    b2Vec2 lastp;
}


static void sResizeWindow(GLFWwindow *, int width, int height) {
    g_camera.m_width = width;
    g_camera.m_height = height;
}

//
static void sKeyCallback(GLFWwindow *, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        switch (key) {


            case GLFW_KEY_ENTER:
                // Quit
                test->EnterKeyDown();
                break;

            case GLFW_KEY_ESCAPE:
                // Quit
                glfwSetWindowShouldClose(mainWindow, GL_TRUE);
                break;

            case GLFW_KEY_LEFT:
                // Pan left
                if (mods == GLFW_MOD_CONTROL) {
                    b2Vec2 newOrigin(2.0f, 0.0f);
                    test->ShiftOrigin(newOrigin);
                }
                else {
                    g_camera.m_center.x -= 0.5f;
                }
                break;

            case GLFW_KEY_RIGHT:
                // Pan right
                if (mods == GLFW_MOD_CONTROL) {
                    b2Vec2 newOrigin(-2.0f, 0.0f);
                    test->ShiftOrigin(newOrigin);
                }
                else {
                    g_camera.m_center.x += 0.5f;
                }
                break;

            case GLFW_KEY_DOWN:
                // Pan down
                if (mods == GLFW_MOD_CONTROL) {
                    b2Vec2 newOrigin(0.0f, 2.0f);
                    test->ShiftOrigin(newOrigin);
                }
                else {
                    g_camera.m_center.y -= 0.5f;
                }
                break;

            case GLFW_KEY_UP:
                // Pan up
                if (mods == GLFW_MOD_CONTROL) {
                    b2Vec2 newOrigin(0.0f, -2.0f);
                    test->ShiftOrigin(newOrigin);
                }
                else {
                    g_camera.m_center.y += 0.5f;
                }
                break;

            case GLFW_KEY_HOME:
                // Reset view
                g_camera.m_zoom = 1.0f;
                g_camera.m_center.Set(0.0f, 20.0f);
                break;

            case GLFW_KEY_Z:
                // Zoom out
                g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
                break;

            case GLFW_KEY_X:
                // Zoom in
                g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
                break;

            case GLFW_KEY_R:
                // Reset test
                delete test;
                test = entry->createFcn();
                break;


            case GLFW_KEY_SPACE:
                // Pause
                settings.pause = !settings.pause;
                break;

            case GLFW_KEY_LEFT_BRACKET:
                // Switch to previous test
                --testSelection;
                if (testSelection < 0) {
                    testSelection = testCount - 1;
                }
                break;

            case GLFW_KEY_RIGHT_BRACKET:
                // Switch to next test
                ++testSelection;
                if (testSelection == testCount) {
                    testSelection = 0;
                }
                break;

            case GLFW_KEY_TAB:
                ui.showMenu = !ui.showMenu;

            default:
                if (test) {
                    test->Keyboard(key);
                }
        }

    }
    /*
    if (action == GLFW_REPEAT) {
        test->Keyboard(key);
    }
     */
    else if (action == GLFW_RELEASE) {
        test->KeyboardUp(key);
    }

}

//
static void sMouseButton(GLFWwindow *, int32 button, int32 action, int32 mods) {
    double xd, yd;
    glfwGetCursorPos(mainWindow, &xd, &yd);
    b2Vec2 ps((float32) xd, (float32) yd);

    // Use the mouse to move things around.
    if (button == GLFW_MOUSE_BUTTON_1) {
        //<##>
        //ps.Set(0, 0);
        b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
        if (action == GLFW_PRESS) {
            if (mods == GLFW_MOD_SHIFT) {
                test->ShiftMouseDown(pw);
            }
            else {
                test->MouseDown(pw);
            }
        }

        if (action == GLFW_RELEASE) {
            test->MouseUp(pw);
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_2) {
        if (action == GLFW_PRESS) {
            lastp = g_camera.ConvertScreenToWorld(ps);
            rightMouseDown = true;
            b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
            test->RightMouseDown(pw);
        }

        if (action == GLFW_RELEASE) {
            rightMouseDown = false;
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_3) {
        b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);

        if (action == GLFW_PRESS) {
            test->MiddleMouseDown(pw);
        }
    }

    }

//
static void sMouseMotion(GLFWwindow *, double xd, double yd) {
    b2Vec2 ps((float) xd, (float) yd);

    b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
    test->MouseMove(pw);

    if (rightMouseDown) {
        b2Vec2 diff = pw - lastp;
        g_camera.m_center.x -= diff.x;
        g_camera.m_center.y -= diff.y;
        lastp = g_camera.ConvertScreenToWorld(ps);
    }
}

//
static void sScrollCallback(GLFWwindow *, double, double dy) {
    if (ui.mouseOverMenu) {
        ui.scroll = -int(dy);
    }
    else {
        if (dy > 0) {
            g_camera.m_zoom /= 1.1f;
        }
        else {
            g_camera.m_zoom *= 1.1f;
        }
    }
}

//
static void sRestart() {
    delete test;
    entry = g_testEntries + testIndex;
    test = entry->createFcn();
}

//
static void sSimulate() {
    glEnable(GL_DEPTH_TEST);
    test->Step(&settings);

    test->DrawTitle(entry->name);

    //((Car *) test)->plotGraphs();
    glDisable(GL_DEPTH_TEST);

    if (testSelection != testIndex) {
        testIndex = testSelection;
        delete test;
        entry = g_testEntries + testIndex;
        test = entry->createFcn();
        g_camera.m_zoom = 1.0f;
        g_camera.m_center.Set(0.0f, 20.0f);
    }
}

//
static void sInterface() {

    bool show_another_window = true;
    int menuWidth = 200;
    ui.mouseOverMenu = false;
    /*
    if (ui.showMenu) {
        ImGui::SetNextWindowSize(ImVec2(200, 100), ImGuiSetCond_FirstUseEver);
        ImGui::Begin("Another Window", &show_another_window);
        ImGui::Text("Hello");
        ImGui::End();
    }
     */


    ImGui::SetNextWindowSize(ImVec2(250, 250), ImGuiSetCond_FirstUseEver);
    bool over = ImGui::Begin("Testbed Controls");
    if (over) ui.mouseOverMenu = true;

    ImGui::Separator();

    //ImGui::LabelText("Test",);
if (ImGui::Button(entry->name))
{
    ui.chooseTest = !ui.chooseTest;
}

    ImGui::Separator();

    ImGui::SliderInt("Vel Iters", &settings.velocityIterations, 0, 50 );
    ImGui::SliderInt("Pos Iters", &settings.positionIterations, 0, 50 );
    ImGui::SliderFloat("Hertz", &settings.hz, 5.0f, 200.0f);

    ImGui::Checkbox("Contact Points", &settings.drawContactPoints);

    ImGui::Checkbox("Contact Normals", &settings.drawContactNormals);

    ImGui::Checkbox("Contact Impulses", &settings.drawContactImpulse);

    ImGui::Checkbox("Friction Impulses", &settings.drawFrictionImpulse);

	ImGui::Checkbox("Single step", &settings.singleStep);


    ImGui::End();

    /*

if (imguiCheck("Sleep", settings.enableSleep, true))
    settings.enableSleep = !settings.enableSleep;
if (imguiCheck("Warm Starting", settings.enableWarmStarting, true))
    settings.enableWarmStarting = !settings.enableWarmStarting;
if (imguiCheck("Time of Impact", settings.enableContinuous, true))
    settings.enableContinuous = !settings.enableContinuous;
if (imguiCheck("Sub-Stepping", settings.enableSubStepping, true))
    settings.enableSubStepping = !settings.enableSubStepping;

imguiSeparatorLine();

if (imguiCheck("Shapes", settings.drawShapes, true))
    settings.drawShapes = !settings.drawShapes;
if (imguiCheck("Joints", settings.drawJoints, true))
    settings.drawJoints = !settings.drawJoints;
if (imguiCheck("AABBs", settings.drawAABBs, true))
    settings.drawAABBs = !settings.drawAABBs;
if (imguiCheck("Contact Points", settings.drawContactPoints, true))
    settings.drawContactPoints = !settings.drawContactPoints;
if (imguiCheck("Contact Normals", settings.drawContactNormals, true))
    settings.drawContactNormals = !settings.drawContactNormals;
if (imguiCheck("Contact Impulses", settings.drawContactImpulse, true))
    settings.drawContactImpulse = !settings.drawContactImpulse;
if (imguiCheck("Friction Impulses", settings.drawFrictionImpulse, true))
    settings.drawFrictionImpulse = !settings.drawFrictionImpulse;
if (imguiCheck("Center of Masses", settings.drawCOMs, true))
    settings.drawCOMs = !settings.drawCOMs;
if (imguiCheck("Statistics", settings.drawStats, true))
    settings.drawStats = !settings.drawStats;
if (imguiCheck("Profile", settings.drawProfile, true))
    settings.drawProfile = !settings.drawProfile;

if (imguiButton("Pause", true))
    settings.pause = !settings.pause;

if (imguiButton("Single Step", true))
    settings.singleStep = !settings.singleStep;

if (imguiButton("Restart", true))
    sRestart();

if (imguiButton("Quit", true))
    glfwSetWindowShouldClose(mainWindow, GL_TRUE);

imguiEndScrollArea();
}

int testMenuWidth = 200;
if (ui.chooseTest)
{
static int testScroll = 0;
bool over = imguiBeginScrollArea("Choose Sample", g_camera.m_width - menuWidth - testMenuWidth - 20, 10, testMenuWidth, g_camera.m_height - 20, &testScroll);
if (over) ui.mouseOverMenu = true;

for (int i = 0; i < testCount; ++i)
{
    if (imguiItem(g_testEntries[i].name, true))
    {
        delete test;
        entry = g_testEntries + i;
        test = entry->createFcn();
        ui.chooseTest = false;
    }
}

imguiEndScrollArea();
}

imguiEndFrame();
     */
}

static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}
//
int main(int argc, char **argv) {
#if defined(_WIN32)
    // Enable memory-leak reports
    _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit()) {
		fprintf(stderr, "Failed to initialize GLFW\n");
		return 1;
	}
		

    const char* glsl_version = "#version 410";
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    g_camera.m_width = 1024;
    g_camera.m_height = 640;


    char title[64];
    sprintf(title, "Cubeswarm");



    mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, NULL, NULL);

	glfwMakeContextCurrent(mainWindow);
	glfwSwapInterval(1); // Enable vsync

	bool err = gladLoadGL() == 0;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls




    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
    ImGui_ImplOpenGL3_Init(glsl_version);



    if (mainWindow == NULL) {
        fprintf(stderr, "Failed to open GLFW mainWindow.\n");
        glfwTerminate();
        return -1;
    }

    printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

    glfwSetScrollCallback(mainWindow, sScrollCallback);
    glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
    glfwSetKeyCallback(mainWindow, sKeyCallback);
    //glfwSetInputMode(mainWindow, GLFW_STICKY_KEYS, 1);
    glfwSetMouseButtonCallback(mainWindow, sMouseButton);
    glfwSetCursorPosCallback(mainWindow, sMouseMotion);
    glfwSetScrollCallback(mainWindow, sScrollCallback);


#if defined(__APPLE__) == FALSE
    //glewExperimental = GL_TRUE;
    //GLenum err = glewInit();
    //if (GLEW_OK != err) {
    //    fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
    //    exit(EXIT_FAILURE);
    //}
#endif

    g_debugDraw.Create();

    testCount = 0;
    while (g_testEntries[testCount].createFcn != NULL) {
        ++testCount;
    }

    testIndex = b2Clamp(testIndex, 0, testCount - 1);
    testSelection = testIndex;

    entry = g_testEntries + testIndex;
    test = entry->createFcn();

    // Control the frame rate. One draw per monitor refresh.
    glfwSwapInterval(1);

    double time1 = glfwGetTime();
    double frameTime = 0.0;

    glClearColor(0.3f, 0.3f, 0.3f, 1.f);


    while (!glfwWindowShouldClose(mainWindow)) {
        glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
        glViewport(0, 0, g_camera.m_width, g_camera.m_height);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        unsigned char mousebutton = 0;
        int mscroll = ui.scroll;
        ui.scroll = 0;

        double xd, yd;
        glfwGetCursorPos(mainWindow, &xd, &yd);
        int mousex = int(xd);
        int mousey = int(yd);

        mousey = g_camera.m_height - mousey;
        int leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);


        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();



        // 1. Show a simple window
        // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"

        sSimulate();
        sInterface();

        // Measure speed
        double time2 = glfwGetTime();
        double alpha = 0.9f;
        frameTime = alpha * frameTime + (1.0 - alpha) * (time2 - time1);
        time1 = time2;

        char buffer[32];
        snprintf(buffer, 32, "%.1f ms", 1000.0 * frameTime);
        //AddGfxCmdText(5, 5, TEXT_ALIGN_LEFT, buffer, WHITE);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_DEPTH_TEST);
        //RenderGLFlush(g_camera.m_width, g_camera.m_height);
		ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        //ImGui::Render();
        glfwSwapBuffers(mainWindow);


        glfwPollEvents();





        int state = glfwGetKey(mainWindow, GLFW_KEY_A);
        if (state == GLFW_PRESS)
        {
            test->handleFastInput(1.f);
        }

        state = glfwGetKey(mainWindow, GLFW_KEY_D);
        if (state == GLFW_PRESS)
        {
            test->handleFastInput(-1.f);
        }


    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    g_debugDraw.Destroy();

    glfwTerminate();

    return 0;
}
