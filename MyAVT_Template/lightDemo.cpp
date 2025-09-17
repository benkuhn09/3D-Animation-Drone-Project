 //
// AVT 2025: Texturing with Phong Shading and Text rendered with TrueType library
// The text rendering was based on https://dev.to/shreyaspranav/how-to-render-truetype-fonts-in-opengl-using-stbtruetypeh-1p5k
// You can also learn an alternative with FreeType text: https://learnopengl.com/In-Practice/Text-Rendering
// This demo was built for learning purposes only.
// Some code could be severely optimised, but I tried to
// keep as simple and clear as possible.
//
// The code comes with no warranties, use it at your own risk.
// You may use it, or parts of it, wherever you want.
// 
// Author: João Madeiras Pereira
//

#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

// include GLEW to access OpenGL 3.3 functions
#include <GL/glew.h>

// GLUT is the toolkit to interface with the OS
#include <GL/freeglut.h>

#include <IL/il.h>

#include "renderer.h"
#include "shader.h"
#include "mathUtility.h"
#include "model.h"
#include "texture.h"

using namespace std;

#define CAPTION "AVT 2025 Welcome Demo"
int WindowHandle = 0;
int WinX = 1024, WinY = 768;

unsigned int FrameCount = 0;

//File with the font
const string fontPathFile = "fonts/arial.ttf";

//Object of class gmu (Graphics Math Utility) to manage math and matrix operations
gmu mu;

//Object of class renderer to manage the rendering of meshes and ttf-based bitmap text
Renderer renderer;
	
// Camera Position
float camX, camY, camZ;

// Camera Spherical Coordinates
float alpha = 57.0f, beta = 18.0f;
float r = 45.0f;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Frame counting and FPS computation
long myTime,timebase = 0,frame = 0;
char s[32];

float lightPos[4] = {4.0f, 5.0f, 2.0f, 1.0f};
//float lightPos[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

//Spotlight
bool spotlight_mode = false;
float coneDir[4] = { 0.0f, -0.0f, -1.0f, 0.0f };

bool fontLoaded = false;

float aspectRatio = 1.0f;
struct Camera {
	float pos[3] = { 0.0f, 0.0f, 0.0f };
	float target[3] = { 0.0f, 0.0f, 0.0f };
	int type = 0; // 0 = perspective, 1 = orthographic
};
Camera cams[3];
int activeCam = 0;

struct Drone {
	float pos[3] = { 0.0f, 15.0f, 0.0f }; // world position (x,y,z)
	float dirAngle = 0.0f;   // yaw (degrees) — heading in the XZ plane
	float pitch = 0.0f;   // nose up/down (degrees) — used to influence forward motion
	float roll = 0.0f;   // roll (degrees) — optional for visuals
	float speed = 0.0f;   // horizontal speed scalar (units/sec)
	float vSpeed = 0.0f;   // vertical speed (units/sec, positive = up)

	// tuning parameters (optional)
	float maxSpeed = 10.0f;
	float maxVSpeed = 8.0f;
	float turnRate = 90.0f;      // degrees/sec when turning (you can override per-key)
	float pitchRate = 30.0f;     // degrees/sec when pitching
};

Drone drone;

struct Building {
	float height;
	int meshID;
};

const int GRID_SIZE = 6;
Building city[GRID_SIZE][GRID_SIZE];

/// ::::::::::::::::::::::::::::::::::::::::::::::::CALLBACK FUNCIONS:::::::::::::::::::::::::::::::::::::::::::::::::://///

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << WinX << "x" << WinY << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
    FrameCount = 0;
    glutTimerFunc(1000, timer, 0);
}

void refresh(int value)
{
	//PUT YOUR CODE HERE
}

// ------------------------------------------------------------
//
// Reshape Callback Function
//

/*void changeSize(int w, int h) {

	float ratio;
	// Prevent a divide by zero, when window is too short
	if(h == 0)
		h = 1;
	// set the viewport to be the entire window
	glViewport(0, 0, w, h);
	// set the projection matrix
	ratio = (1.0f * w) / h;
	mu.loadIdentity(gmu::PROJECTION);
	mu.perspective(53.13f, ratio, 0.1f, 1000.0f);
}*/

void changeSize(int w, int h) {
	if (h == 0) h = 1;  // prevent divide by zero

	// set viewport
	glViewport(0, 0, w, h);

	// update aspect ratio for later use in renderSim
	aspectRatio = static_cast<float>(w) / static_cast<float>(h);
}


// ------------------------------------------------------------
//
// Render stufff
//

void initCity() {

	float minHeight = 4.0f;   // minimum building height
	float maxHeight = 8.5f;  // maximum building height

	for (int i = 0; i < GRID_SIZE; ++i) {
		for (int j = 0; j < GRID_SIZE; ++j) {
			float t = rand() / (float)RAND_MAX;  // random float 0–1
			city[i][j].height = minHeight + t * (maxHeight - minHeight);
			city[i][j].meshID = (rand() % 2 == 0) ? 1 : 3;  // cube or cylinder
		}
	}
}

void updateDrone(float deltaTime) {
	// convert angles to radians
	float yawRad = drone.dirAngle * 3.14f / 180.0f;
	float pitchRad = drone.pitch * 3.14f / 180.0f;

	// direction vector (XZ plane)
	float dx = cos(pitchRad) * sin(yawRad);
	float dz = cos(pitchRad) * cos(yawRad);
	float dy = sin(pitchRad);

	// update position
	drone.pos[0] += dx * drone.speed * deltaTime;
	drone.pos[1] += drone.vSpeed * deltaTime + dy * drone.speed * deltaTime;
	drone.pos[2] += dz * drone.speed * deltaTime;

	// prevent drone from going below the floor
	if (drone.pos[1] < 1.0f) drone.pos[1] = 1.0f;
}

float lastTime = 0.0f;

void renderSim(void) {

	FrameCount++;

	float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float deltaTime = currentTime - lastTime;
	lastTime = currentTime;
	updateDrone(deltaTime);

	float distance = 20.0f; // how far behind
	float height = 10.0f;  // how far above
	float yawRad = drone.dirAngle * 3.14f / 180.0f;

	cams[2].pos[0] = drone.pos[0] - sin(yawRad) * distance;
	cams[2].pos[1] = drone.pos[1] + height;
	cams[2].pos[2] = drone.pos[2] - cos(yawRad) * distance;

	cams[2].target[0] = drone.pos[0];
	cams[2].target[1] = drone.pos[1];
	cams[2].target[2] = drone.pos[2];

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderer.activateRenderMeshesShaderProg(); // use the required GLSL program to draw the meshes with illumination

	//Associar os Texture Units aos Objects Texture
	//stone.tga loaded in TU0; checker.tga loaded in TU1;  lightwood.tga loaded in TU2
	renderer.setTexUnit(0, 0);
	renderer.setTexUnit(1, 1);
	renderer.setTexUnit(2, 2);

	// load identity matrices
	mu.loadIdentity(gmu::VIEW);
	mu.loadIdentity(gmu::MODEL);
	// set the camera using a function similar to gluLookAt
	//mu.lookAt(camX, camY, camZ, 0, 0, 0, 0, 1, 0);
	mu.loadIdentity(gmu::PROJECTION);
	Camera& cam = cams[activeCam];
	// switch between perspective vs ortho
	if (cam.type == 0) {
		// perspective camera
		mu.perspective(53.13f, aspectRatio, 0.1f, 1000.0f);
	}
	else {
		// orthographic camera
		float size = 30.0f;  // zoom level
		mu.ortho(-size * aspectRatio, size * aspectRatio,
			-size, size, 0.1f, 1000.0f);
	}
	// then apply the camera view transform
	mu.loadIdentity(gmu::VIEW);
	mu.lookAt(cam.pos[0], cam.pos[1], cam.pos[2],
		cam.target[0], cam.target[1], cam.target[2],
		0, 1, 0);


	//send the light position in eye coordinates
	//renderer.setLightPos(lightPos); //efeito capacete do mineiro, ou seja lighPos foi definido em eye coord 

	float lposAux[4];
	mu.multMatrixPoint(gmu::VIEW, lightPos, lposAux);   //lightPos definido em World Coord so is converted to eye space
	renderer.setLightPos(lposAux);

	//Spotlight settings
	renderer.setSpotLightMode(spotlight_mode);
	renderer.setSpotParam(coneDir, 0.93);

	dataMesh data;
	
	// Draw the floor - myMeshes[0] contains the quad object
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, 0.0f, -0.5f, 0.0f);
	//mu.scale(gmu::MODEL, 30.0f, 0.1f, 30.0f);
	mu.rotate(gmu::MODEL, -90.0f, 1.0f, 0.0f, 0.0f);
	mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f); //centrar o cubo na origem

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	data.meshID = 0;
	data.texMode = 1; //modulate diffuse color with texel color
	data.vm = mu.get(gmu::VIEW_MODEL),
	data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	data.normal = mu.getNormalMatrix();
	renderer.renderMesh(data);
	mu.popMatrix(gmu::MODEL);

	float spacing = 7.2f;
	float floorSize = 45.0f;
	float xOffset = -floorSize / 2.0f + spacing / 2.0f;
	float zOffset = -floorSize / 2.0f + spacing / 2.0f;

	//Draw the other objects
	int objId = 1; //id of the current object mesh - to be used as index of the array Mymeshes in the renderer object
	for (int i = 0; i < GRID_SIZE; ++i) {
		for (int j = 0; j < GRID_SIZE; ++j) {
			mu.pushMatrix(gmu::MODEL);

			//mu.translate(gmu::MODEL, (float)i * 5.0f, 0.0f, (float)j * 5.0f);
			// move grid so that (0,0) starts at top-left of floor

			mu.translate(gmu::MODEL,
				xOffset + (float)i * spacing,
				0.0f,
				zOffset + (float)j * spacing);
			
			//retrives height from cityInit
			float height = city[i][j].height;
			
			
			//mu.scale(gmu::MODEL, 2.3f, height, 2.3f);

			/*if (city[i][j].meshID == 3) {
				mu.translate(gmu::MODEL, 0.0f, 0.5f, 0.0f);
			}*/

			//mu.scale(gmu::MODEL, 2.3f, height, 2.3f);

			if (city[i][j].meshID == 3) {
				// first scale
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
				// then translate up by 0.5 in *scaled units* (i.e., after scaling the cylinder)
				mu.translate(gmu::MODEL, 0.0f, 0.5f, 0.0f);  // 0.5 *after scale ensures bottom is at 0
			}
			else {
				// cubes: just scale
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
			}


			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();

			//data.meshID = objId;
			//data.meshID = (rand() % 2 == 0) ? 1 : 3;
			data.meshID = city[i][j].meshID;
			data.texMode = i;   //0:no texturing; 1:modulate diffuse color with texel color; 2:diffuse color is replaced by texel color; 3: multitexturing
			data.vm = mu.get(gmu::VIEW_MODEL),
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);

			mu.popMatrix(gmu::MODEL);
			//if (objId == 3) objId = 1;
			//else objId++;
		}
	}

	// drone rendering
	/*float droneX = 0.0f;
	float droneY = 15.0f;
	float droneZ = 0.0f;
	float droneScale = 1.0f;

	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, droneX, droneY, droneZ);
	mu.scale(gmu::MODEL, droneScale, droneScale, droneScale);*/

	float droneScale = 1.0f;

	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
	mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f);  // yaw rotation
	mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f);     // pitch rotation
	mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f);      // roll rotation
	mu.scale(gmu::MODEL, droneScale, droneScale, droneScale);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	data.meshID = 2;  // sphere mesh, for now
	data.texMode = 0; //no texturing
	data.vm = mu.get(gmu::VIEW_MODEL),
	data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	data.normal = mu.getNormalMatrix();
	renderer.renderMesh(data);
	mu.popMatrix(gmu::MODEL);


	//Render text (bitmap fonts) in screen coordinates. So use ortoghonal projection with viewport coordinates.
	//Each glyph quad texture needs just one byte color channel: 0 in background and 1 for the actual character pixels. Use it for alpha blending
	//text to be rendered in last place to be in front of everything
	
	if(fontLoaded) {
		glDisable(GL_DEPTH_TEST);
		TextCommand textCmd = { "2025 Drone Project", {100, 200}, 0.5 };
		//the glyph contains transparent background colors and non-transparent for the actual character pixels. So we use the blending
		glEnable(GL_BLEND);  
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		int m_viewport[4];
		glGetIntegerv(GL_VIEWPORT, m_viewport);

		//viewer at origin looking down at  negative z direction

		mu.loadIdentity(gmu::MODEL);
		mu.loadIdentity(gmu::VIEW);
		mu.pushMatrix(gmu::PROJECTION);
		mu.loadIdentity(gmu::PROJECTION);
		mu.ortho(m_viewport[0], m_viewport[0] + m_viewport[2] - 1, m_viewport[1], m_viewport[1] + m_viewport[3] - 1, -1, 1);
		mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
		textCmd.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
		renderer.renderText(textCmd);
		mu.popMatrix(gmu::PROJECTION);
		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		
	}
	
	glutSwapBuffers();
}

// ------------------------------------------------------------
//
// Events from the Keyboard
//

void processKeys(unsigned char key, int xx, int yy)
{
	switch(key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'c': 
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", alpha, beta, r);
			break;

		case 'l':   //toggle spotlight mode
			if (!spotlight_mode) {
				spotlight_mode = true;
				printf("Point light disabled. Spot light enabled\n");
			}
			else {
				spotlight_mode = false;
				printf("Spot light disabled. Point light enabled\n");
			}
			break;

		case 'r':    //reset
			alpha = 57.0f; beta = 18.0f;  // Camera Spherical Coordinates
			r = 45.0f;
			camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
			camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
			camY = r * sin(beta * 3.14f / 180.0f);
			break;

		case 'm': glEnable(GL_MULTISAMPLE); break;
		case 'n': glDisable(GL_MULTISAMPLE); break;

		case '1': activeCam = 0; break; // top-persp
		case '2': activeCam = 1; break; // top-ortho
		case '3': activeCam = 2; break; // follow-drone

		case 't': // throttle up
			drone.vSpeed += 0.1f;
			break;
		case 'g': // throttle down
			drone.vSpeed -= 0.1f;
			break;
		case '+': // increase forward speed
			drone.speed += 0.1f;
			break;
		case '-': // decrease forward speed
			drone.speed -= 0.1f;
			//if (drone.speed < 0.0f) drone.speed = 0.0f;
			break;

	}
}

void processSpecialKeys(int key, int xx, int yy) {
	switch (key) {
	case GLUT_KEY_UP:    // pitch forward
		drone.pitch += 2.0f;
		break;
	case GLUT_KEY_DOWN:  // pitch backward
		drone.pitch -= 2.0f;
		break;
	case GLUT_KEY_LEFT:  // yaw left
		drone.dirAngle -= 5.0f;
		break;
	case GLUT_KEY_RIGHT: // yaw right
		drone.dirAngle += 5.0f;
		break;
	}
}

// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN)  {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX =  - xx + startX;
	deltaY =    yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	//camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	//camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	//camY = rAux *   						       sin(betaAux * 3.14f / 180.0f);

	// spherical camera math
	float camXtemp = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	float camZtemp = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	float camYtemp = rAux * sin(betaAux * 3.14f / 180.0f);

	/*if (activeCam == 2) {
		// follow drone
		float distance = 20.0f; // behind the drone
		float height = 10.0f; // camera height above drone
		float targetHeight = 2.0f; // where camera looks relative to drone's center
		float yawRad = drone.dirAngle * 3.14159f / 180.0f;

		// Camera position behind and above the drone
		cams[2].pos[0] = drone.pos[0] - sin(yawRad) * distance;
		cams[2].pos[1] = drone.pos[1] + height;
		cams[2].pos[2] = drone.pos[2] - cos(yawRad) * distance;

		// Camera target slightly above drone base (so it looks forward, not down)
		cams[2].target[0] = drone.pos[0];
		cams[2].target[1] = drone.pos[1] + targetHeight;
		cams[2].target[2] = drone.pos[2];
	}*/
	
		// free orbit cameras (0 or 1)
		camX = camXtemp;
		camY = camYtemp;
		camZ = camZtemp;

		cams[activeCam].pos[0] = camX;
		cams[activeCam].pos[1] = camY;
		cams[activeCam].pos[2] = camZ;

		cams[activeCam].target[0] = 0.0f;
		cams[activeCam].target[1] = 0.0f;
		cams[activeCam].target[2] = 0.0f;
	


//  uncomment this if not using an idle or refresh func
//	glutPostRedisplay();
}


void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r *   						     sin(beta * 3.14f / 180.0f);

//  uncomment this if not using an idle or refresh func
//	glutPostRedisplay();
}


//
// Scene building with basic geometry
//

void buildScene()
{
	//Texture Object definition
	renderer.TexObjArray.texture2D_Loader("assets/stone.tga");
	renderer.TexObjArray.texture2D_Loader("assets/checker.png");
	renderer.TexObjArray.texture2D_Loader("assets/lightwood.tga");

	//Scene geometry with triangle meshes

	MyMesh amesh;

	float amb[] = { 0.2f, 0.15f, 0.1f, 1.0f };
	float diff[] = { 0.8f, 0.6f, 0.4f, 1.0f };
	float spec[] = { 0.8f, 0.8f, 0.8f, 1.0f };

	float amb1[] = { 0.3f, 0.0f, 0.0f, 1.0f };
	float diff1[] = { 0.8f, 0.1f, 0.1f, 1.0f };
	float spec1[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	float emissive[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float shininess = 100.0f;
	int texcount = 0;

	// create geometry and VAO of the floor quad
	amesh = createQuad(45.0f, 45.0f);
	memcpy(amesh.mat.ambient, amb1, 4 * sizeof(float));
	memcpy(amesh.mat.diffuse, diff1, 4 * sizeof(float));
	memcpy(amesh.mat.specular, spec1, 4 * sizeof(float));
	memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	amesh.mat.shininess = shininess;
	amesh.mat.texCount = texcount;
	renderer.myMeshes.push_back(amesh);

	
	// create geometry and VAO of the cube
	amesh = createCube();
	memcpy(amesh.mat.ambient, amb1, 4 * sizeof(float));
	memcpy(amesh.mat.diffuse, diff1, 4 * sizeof(float));
	memcpy(amesh.mat.specular, spec1, 4 * sizeof(float));
	memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	amesh.mat.shininess = shininess;
	amesh.mat.texCount = texcount;
	renderer.myMeshes.push_back(amesh);

	// create geometry and VAO of the pawn
	//amesh = createPawn();
	//memcpy(amesh.mat.ambient, amb, 4 * sizeof(float));
	//memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	//memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	//memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	//amesh.mat.shininess = shininess;
	//amesh.mat.texCount = texcount;
	//renderer.myMeshes.push_back(amesh);

	// create geometry and VAO of the sphere
	amesh = createSphere(1.0f, 20);
	memcpy(amesh.mat.ambient, amb, 4 * sizeof(float));
	memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	amesh.mat.shininess = shininess;
	amesh.mat.texCount = texcount;
	renderer.myMeshes.push_back(amesh);

	// create geometry and VAO of the cylinder
	amesh = createCylinder(1.5f, 0.5f, 20);
	memcpy(amesh.mat.ambient, amb, 4 * sizeof(float));
	memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	amesh.mat.shininess = shininess;
	amesh.mat.texCount = texcount;
	renderer.myMeshes.push_back(amesh);

	// create geometry and VAO of the cone
	//amesh = createCone(2.5f, 1.2f, 20);
	//memcpy(amesh.mat.ambient, amb, 4 * sizeof(float));
	//memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	//memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	//memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	//amesh.mat.shininess = shininess;
	//amesh.mat.texCount = texcount;
	//renderer.myMeshes.push_back(amesh);

	// create geometry and VAO of the torus
	//amesh = createTorus(0.5f, 1.5f, 20, 20);
	//memcpy(amesh.mat.ambient, amb, 4 * sizeof(float));
	//memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	//memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	//memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	//amesh.mat.shininess = shininess;
	//amesh.mat.texCount = texcount;
	//renderer.myMeshes.push_back(amesh);

	//The truetypeInit creates a texture object in TexObjArray for storing the fontAtlasTexture
	
	fontLoaded = renderer.truetypeInit(fontPathFile);
	if (!fontLoaded)
		cerr << "Fonts not loaded\n";
	else 
		cerr << "Fonts loaded\n";

	printf("\nNumber of Texture Objects is %d\n\n", renderer.TexObjArray.getNumTextureObjects());

	srand(time(NULL));  // different city each run
	initCity();         // fill city grid with random data


	// set the camera position based on its spherical coordinates
	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);

	// top-down perspective
	cams[0].pos[0] = 0.0f;
	cams[0].pos[1] = 50.0f;     // high above
	cams[0].pos[2] = 0.01f;     // slight offset to avoid singularity
	cams[0].target[0] = 0.0f;
	cams[0].target[1] = 0.0f;
	cams[0].target[2] = 0.0f;
	cams[0].type = 0;

	// top-down orthographic
	cams[1] = cams[0];          // copy same pos/target
	cams[1].type = 1;

	// follow drone
	//float droneX = 0.0f, droneY = 15.0f, droneZ = 0.0f;
	cams[2].target[0] = drone.pos[0];
	cams[2].target[1] = drone.pos[1];
	cams[2].target[2] = drone.pos[2];

	cams[2].pos[0] = drone.pos[0];
	cams[2].pos[1] = drone.pos[1] + 5.0f;   // slightly above
	cams[2].pos[2] = drone.pos[2] + 20.0f;  // behind
	cams[2].type = 0;                       // perspective

	activeCam = 0; // default camera is top-down perspective
}

// ------------------------------------------------------------
//
// Main function
//

int main(int argc, char **argv) {

//  GLUT initialization
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA|GLUT_MULTISAMPLE);

	glutInitContextVersion (4, 3);
	glutInitContextProfile (GLUT_CORE_PROFILE );
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE | GLUT_DEBUG);

	glutInitWindowPosition(100,100);
	glutInitWindowSize(WinX, WinY);
	WindowHandle = glutCreateWindow(CAPTION);

//  Callback Registration
	glutDisplayFunc(renderSim);
	glutReshapeFunc(changeSize);

	glutTimerFunc(0, timer, 0);
	glutIdleFunc(renderSim);  // Use it for maximum performance
	//glutTimerFunc(0, refresh, 0);    //use it to to get 60 FPS whatever

//	Mouse and Keyboard Callbacks
	glutKeyboardFunc(processKeys);
	glutSpecialFunc(processSpecialKeys);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc ( mouseWheel ) ;
	

//	return from main loop
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

//	Init GLEW
	glewExperimental = GL_TRUE;
	glewInit();

	// some GL settings
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));

	/* Initialization of DevIL */
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	buildScene();

	if(!renderer.setRenderMeshesShaderProg("shaders/mesh.vert", "shaders/mesh.frag") || 
		!renderer.setRenderTextShaderProg("shaders/ttf.vert", "shaders/ttf.frag"))
	return(1);

	//  GLUT main loop
	glutMainLoop(); // infinite loop

	return(0);
}



