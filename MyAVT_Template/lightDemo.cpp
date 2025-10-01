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
#include <algorithm>

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
float alpha = 57.0f, beta = 18.0f;
float r = 45.0f;

// ACSII array for key states
bool keys[256] = { false };
bool specialKeys[256] = { false };

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Frame counting and FPS computation
long myTime,timebase = 0,frame = 0;
char s[32];

//float lightPos[4] = {4.0f, 5.0f, 2.0f, 1.0f};
//float lightPos[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

//Spotlight
//bool spotlight_mode = false;
//float coneDir[4] = { 0.0f, -0.0f, -1.0f, 0.0f };

bool fontLoaded = false;

// fog flag
bool fogEnabled = true;

float aspectRatio = 1.0f;

bool debugDrawFlyingAABB = true;  // toggle with 'b'
int  debugFlyIndex = 0;           // select which flying object, [0..NUM_FLYING_OBJECTS-1]
int  wireCubeMeshID = -1;         // green wire cube used for drawing AABBs



struct Mat4 {
	float m[16]; // column-major
};

static Mat4 I4() {
	Mat4 M{};
	M.m[0] = M.m[5] = M.m[10] = M.m[15] = 1.0f;
	return M;
}

static Mat4 mat4FromGmu(const float* g) {
	Mat4 M{};
	for (int i = 0; i < 16; ++i) M.m[i] = g[i]; // gmu is column-major too
	return M;
}

// transform a point (x,y,z,1)
static void xformPoint(const Mat4& M, const float p[3], float out[3]) {
	out[0] = M.m[0] * p[0] + M.m[4] * p[1] + M.m[8] * p[2] + M.m[12];
	out[1] = M.m[1] * p[0] + M.m[5] * p[1] + M.m[9] * p[2] + M.m[13];
	out[2] = M.m[2] * p[0] + M.m[6] * p[1] + M.m[10] * p[2] + M.m[14];
}

// given a local AABB and a model matrix, compute world AABB
static void aabbFromXformedBox(const float localMin[3], const float localMax[3],
	const Mat4& M, float outMin[3], float outMax[3]) {
	// 8 corners
	float c[8][3] = {
		{localMin[0],localMin[1],localMin[2]},
		{localMax[0],localMin[1],localMin[2]},
		{localMin[0],localMax[1],localMin[2]},
		{localMax[0],localMax[1],localMin[2]},
		{localMin[0],localMin[1],localMax[2]},
		{localMax[0],localMin[1],localMax[2]},
		{localMin[0],localMax[1],localMax[2]},
		{localMax[0],localMax[1],localMax[2]}
	};
	float w[3]; xformPoint(M, c[0], w);
	outMin[0] = outMax[0] = w[0];
	outMin[1] = outMax[1] = w[1];
	outMin[2] = outMax[2] = w[2];
	for (int i = 1;i < 8;++i) {
		xformPoint(M, c[i], w);
		outMin[0] = std::min(outMin[0], w[0]); outMax[0] = std::max(outMax[0], w[0]);
		outMin[1] = std::min(outMin[1], w[1]); outMax[1] = std::max(outMax[1], w[1]);
		outMin[2] = std::min(outMin[2], w[2]); outMax[2] = std::max(outMax[2], w[2]);
	}
}

// Draw a wireframe AABB given min/max in world space using a green cube mesh.
// Assumes renderer meshes shader is active.
/*static void drawWireAABB(const float bmin[3], const float bmax[3]) {
	// scale and translate a unit cube in [0,1]^3
	const float sx = (bmax[0] - bmin[0]);
	const float sy = (bmax[1] - bmin[1]);
	const float sz = (bmax[2] - bmin[2]);

	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, bmin[0], bmin[1], bmin[2]);
	mu.scale(gmu::MODEL, sx, sy, sz);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	dataMesh d{};
	d.meshID = wireCubeMeshID;   // green cube created in buildScene
	d.texMode = 0;
	d.vm = mu.get(gmu::VIEW_MODEL);
	d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	d.normal = mu.getNormalMatrix();

	// draw as wireframe (line mode) + thicker lines, backfaces visible
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(2.0f);

	renderer.renderMesh(d);

	// restore state
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);

	mu.popMatrix(gmu::MODEL);
}*/


/*Lighting Globals*/
float directionalLightPos[4] = { 1.0f, -100.0f, -100.0f, 0.0f };
bool directionalLightOn = true;

float pointLightPos[NUMBER_POINT_LIGHTS][4] = {
	{ 0.0f, 5.0f,  -10.0f, 1.0f},
	{ 10.0f, 5.0f,  10.0f, 1.0f},
	{-10.0f, 5.0f,   0.0f, 1.0f},
	{ 20.0f, 5.0f, -15.0f, 1.0f},
	{-20.0f, 5.0f,  10.0f, 1.0f},
	{  0.0f, 5.0f,  15.0f, 1.0f}
};

float pointLightColor[NUMBER_POINT_LIGHTS][3] = {
	{1.0f, 0.0f, 0.0f}, // red
	{0.0f, 1.0f, 0.0f}, // green
	{0.0f, 0.0f, 1.0f}, // blue
	{1.0f, 1.0f, 0.0f}, // yellow
	{1.0f, 0.0f, 1.0f}, // magenta
	{0.0f, 1.0f, 1.0f}  // cyan
};


bool pointLightsOn = true;
float pointLightEye[NUMBER_POINT_LIGHTS][4];

bool spotLightsOn = true;
float spotCutOff = 0.93f;


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

	// tuning parameters
	float maxSpeed = 10.0f;
	float maxVSpeed = 8.0f;
	float turnRate = 90.0f;      // degrees/sec when turning (you can override per-key)
	float pitchRate = 30.0f;     // degrees/sec when pitching
	float size = 1.0f; // for bounding, in world units

	float pitchVel = 0.0f;  // deg/s
	float rollVel = 0.0f;  // deg/s
};

Drone drone;
float cam2_yawOffset = 0.0f;   // horizontal orbit around drone
float cam2_pitchOffset = 0.0f; // vertical orbit around drone

// --- Attitude limits & dynamics (degrees-based) ---
const float MAX_PITCH = 30.0f;   // how far the nose can tilt up/down
const float MAX_ROLL = 35.0f;   // how far it can bank left/right
const float ANG_INPUT_ACCEL = 160.0f;  // deg/s^2 injected while a key is held
const float ANG_DAMP = 2.5f;    // 1/s   (angular friction)
const float ANG_SPRING = 4.0f;    // 1/s^2 (auto-level pull to 0°)

struct Building {
	float height;
	int meshID;
	int texMode;
};

const int GRID_SIZE = 6;
Building city[GRID_SIZE][GRID_SIZE];

float buildingOffset[GRID_SIZE][GRID_SIZE][3];

struct FlyingObject {
	float pos[3];   // world position
	float dir[3];   // unit direction
	float speed;    // units/sec
	int   meshID;   // which mesh to render

	float rotAngle;   // degrees
	float rotSpeed;   // degrees/sec
	int   rotAxis;

	float size;
};

std::vector<FlyingObject> flyingObjects;

const int   NUM_FLYING_OBJECTS = 8;
const float SPAWN_RADIUS = 30.0f;  // birth ring radius (XZ)
const float KILL_RADIUS = 45.0f;  // die when too far from center
const float MIN_Y = 0.0f;
const float MAX_Y = 20.0f;

const float MAX_XZ = 20.0f;
const float MIN_XZ = -20.0f;


const float BASE_SPEED = 20.0f;   // initial median speed

// Difficulty ramp (~every 30 s)
float speedFactor = 1.0f;
float nextSpeedBump = 5.0f;  // seconds

// for later
int meshFlyingObject = -1;

int droneBodyMeshID = -1;
int droneRotorMeshID = -1;

int glassCubeMeshID = -1;
int glassCylMeshID = -1;


static inline float frand(float a, float b) {
	return a + (b - a) * (rand() / (float)RAND_MAX);
}
static inline void normalize3(float v[3]) {
	float L = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
	if (L > 0.0f) { v[0] /= L; v[1] /= L; v[2] /= L; }
}

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


void spawnFlyingObject(FlyingObject& o) {
	// spawn on an XZ circle at SPAWN_RADIUS, at random height
	const float ang = frand(0.0f, 6.2831853f);
	o.pos[0] = SPAWN_RADIUS * cosf(ang);
	o.pos[2] = SPAWN_RADIUS * sinf(ang);
	o.pos[1] = frand(5, MAX_Y);

	// aim roughly across the city (toward center with random height)
	float target[3] = { frand(MIN_XZ, MAX_XZ), frand(MIN_Y, MAX_Y),frand(MIN_XZ, MAX_XZ) };
	o.dir[0] = target[0] - o.pos[0];
	o.dir[1] = target[1] - o.pos[1];
	o.dir[2] = target[2] - o.pos[2];
	normalize3(o.dir);


	o.rotAxis = (int)frand(0.0f, 3.0f);          // 0,1,or 2
	o.rotSpeed = frand(45.0f, 180.0f);            // 45–180 deg/s
	o.rotAngle = frand(0.0f, 360.0f);
	// vary the speed
	o.speed = BASE_SPEED * frand(0.5f, 1.5f);


	o.size = frand(0.5, 4);
	// for later
	o.meshID = (meshFlyingObject >= 0 ? meshFlyingObject : 0);
}

void initFlyingObjects() {
	flyingObjects.resize(NUM_FLYING_OBJECTS);
	for (auto& o : flyingObjects) spawnFlyingObject(o);
}

void updateFlyingObjects(float dt, float elapsedSeconds) {
	// difficulty bump every ~30 s
	if (elapsedSeconds >= nextSpeedBump) {
		speedFactor *= 1.15f;
		nextSpeedBump += 30.0f;
	}

	for (auto& o : flyingObjects) {
		o.pos[0] += o.dir[0] * o.speed * speedFactor * dt;
		o.pos[1] += o.dir[1] * o.speed * speedFactor * dt;
		o.pos[2] += o.dir[2] * o.speed * speedFactor * dt;

		o.rotAngle += o.rotSpeed * dt;
		// die & respawn when too far from center
		float r2 = o.pos[0] * o.pos[0] + o.pos[1] * o.pos[1] + o.pos[2] * o.pos[2];
		if (r2 > (KILL_RADIUS * KILL_RADIUS)) spawnFlyingObject(o);
	}
}

void refresh(int value)
{
	static float last = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float dt = now - last;
	last = now;

	updateFlyingObjects(dt, now);

	glutPostRedisplay();
	glutTimerFunc(16, refresh, 0);
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
			city[i][j].texMode = 2 + (rand() % 4);
			if (city[i][j].texMode == 4) { // skyscraper_glass.jpg
				city[i][j].meshID = (city[i][j].meshID == 3 ? glassCubeMeshID:glassCylMeshID);
			}
			/* initializing offsets for building. used for collision. */
			buildingOffset[i][j][0] = 0.0f;
			buildingOffset[i][j][1] = 0.0f;
			buildingOffset[i][j][2] = 0.0f;
		}
	}
}

bool aabbIntersect(const float minA[3], const float maxA[3], const float minB[3], const float maxB[3]) {
	if (maxA[0] < minB[0] || minA[0] > maxB[0]) return false;
	if (maxA[1] < minB[1] || minA[1] > maxB[1]) return false;
	if (maxA[2] < minB[2] || minA[2] > maxB[2]) return false;
	return true;
}

void computeBuildingAABB(int i, int j, float outMin[3], float outMax[3]) {
	const float spacing = 7.2f;
	const float floorSize = 45.0f;
	const float scaleX = 2.3f;
	const float scaleZ = 2.3f;

	float xOffset = -floorSize / 2.0f + spacing / 2.0f;
	float zOffset = -floorSize / 2.0f + spacing / 2.0f;

	float baseX = xOffset + i * spacing + buildingOffset[i][j][0];
	float baseZ = zOffset + j * spacing + buildingOffset[i][j][2];
	float h = city[i][j].height;
	float yOff = buildingOffset[i][j][1];

	int mID = city[i][j].meshID;

	if (mID == 3 || mID == glassCylMeshID) {
		// Cylinder: after scale+translate(0,0.5,0)
		outMin[0] = baseX - scaleX * 0.5f;
		outMax[0] = baseX + scaleX * 0.5f;
		outMin[2] = baseZ - scaleZ * 0.5f;
		outMax[2] = baseZ + scaleZ * 0.5f;
		outMin[1] = yOff + 0.5f;     // cylinder bottom lifted by 0.5
		outMax[1] = outMin[1] + h;   // full height above
	}
	else {
		// Cube: spans full [0,scale] instead of centered
		outMin[0] = baseX;
		outMax[0] = baseX + scaleX;
		outMin[2] = baseZ;
		outMax[2] = baseZ + scaleZ;
		outMin[1] = yOff;
		outMax[1] = yOff + h;
	}
}


void computeDroneAABB(const Drone& drone, float outMin[3], float outMax[3]) {
	// shapes in local space
	const float cubeMin[3] = { 0.0f, 0.0f, 0.0f };  // unit cube [0,1]^3
	const float cubeMax[3] = { 1.0f, 1.0f, 1.0f };

	// rotor cylinder after the render's translate(0.5,0,0.5) is applied:
	// X,Z in [0,1], Y in [-0.5, 0.5]
	const float cylMin[3] = { -0.5f, -0.5f, -0.5f };
	const float cylMax[3] = { 0.5f,  0.5f,  0.5f };

	const float bodyScaleX = 2.0f, bodyScaleY = 0.6f, bodyScaleZ = 2.0f;
	const float rotorScaleX = 1.0f, rotorScaleY = 0.3f, rotorScaleZ = 1.0f;
	const float rotorHeight = 0.2f;
	const float halfX = bodyScaleX * 0.5f;
	const float halfZ = bodyScaleZ * 0.5f;

	bool first = true;
	auto accumulateCurrentMuModel = [&](const float lmin[3], const float lmax[3]) {
		Mat4 M = mat4FromGmu(mu.get(gmu::MODEL));
		float wmin[3], wmax[3];
		aabbFromXformedBox(lmin, lmax, M, wmin, wmax);
		if (first) {
			outMin[0] = wmin[0]; outMin[1] = wmin[1]; outMin[2] = wmin[2];
			outMax[0] = wmax[0]; outMax[1] = wmax[1]; outMax[2] = wmax[2];
			first = false;
		}
		else {
			outMin[0] = std::min(outMin[0], wmin[0]);
			outMin[1] = std::min(outMin[1], wmin[1]);
			outMin[2] = std::min(outMin[2], wmin[2]);
			outMax[0] = std::max(outMax[0], wmax[0]);
			outMax[1] = std::max(outMax[1], wmax[1]);
			outMax[2] = std::max(outMax[2], wmax[2]);
		}
		};

	// === Recreate EXACT render path with mu ===
	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);

	// base drone transform (translate + yaw + pitch + roll) — same order as render
	mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
	mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f); // yaw
	mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f); // pitch
	mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f); // roll

	// --- Body (matches renderSim) ---
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f); // center cube
	mu.scale(gmu::MODEL, bodyScaleX, bodyScaleY, bodyScaleZ);
	accumulateCurrentMuModel(cubeMin, cubeMax);
	mu.popMatrix(gmu::MODEL);

	// --- Rotors (4 corners; matches renderSim order) ---
	for (int ix = -1; ix <= 1; ix += 2) {
		for (int iz = -1; iz <= 1; iz += 2) {
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, ix * halfX, rotorHeight, iz * halfZ);
			// render path does this before scaling
			mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
			mu.scale(gmu::MODEL, rotorScaleX, rotorScaleY, rotorScaleZ);
			accumulateCurrentMuModel(cylMin, cylMax);
			mu.popMatrix(gmu::MODEL);
		}
	}

	mu.popMatrix(gmu::MODEL);
}


void computeFlyingObjectAABB(const FlyingObject& o, float outMin[3], float outMax[3]) {
	// Recreate the SAME model xform path that you use to render the object.
	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);

	mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
	if (o.rotAxis == 0)      mu.rotate(gmu::MODEL, o.rotAngle, 1.0f, 0.0f, 0.0f);
	else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 1.0f, 0.0f);
	else                     mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 0.0f, 1.0f);
	mu.scale(gmu::MODEL, o.size, o.size, o.size);

	// Get the exact matrix gmu used
	Mat4 M = mat4FromGmu(mu.get(gmu::MODEL));
	mu.popMatrix(gmu::MODEL);

	// Local unit cube of the flying object (your createCube() is [0,1]^3)
	const float cubeMin[3] = { 0.0f, 0.0f, 0.0f };
	const float cubeMax[3] = { 1.0f, 1.0f, 1.0f };

	aabbFromXformedBox(cubeMin, cubeMax, M, outMin, outMax);
}





bool findCollidingBuilding(const float dMin[3], const float dMax[3], int& outI, int& outJ) {
	for (int i = 0; i < GRID_SIZE; ++i) {
		for (int j = 0; j < GRID_SIZE; ++j) {
			float bMin[3], bMax[3];
			computeBuildingAABB(i, j, bMin, bMax);
			if (aabbIntersect(dMin, dMax, bMin, bMax)) {
				outI = i;
				outJ = j;
				return true;
			}
		}
	}
	return false;
}

void resetDrone() {
	drone.pos[0] = 0.0f;
	drone.pos[1] = 15.0f;
	drone.pos[2] = 0.0f;
	drone.dirAngle = 0.0f;
	drone.pitch = 0.0f;
	drone.roll = 0.0f;
	drone.speed = 0.0f;
	drone.vSpeed = 0.0f;
}

void updateDrone(float deltaTime) {

	float prevPos[3] = { drone.pos[0], drone.pos[1], drone.pos[2] };

	// convert angles to radians
	float yawRad = drone.dirAngle * 3.14159f / 180.0f;
	float pitchRad = drone.pitch * 3.14159f / 180.0f;
	float rollRad = drone.roll * 3.14159f / 180.0f;

	// forward direction (affected by yaw + pitch)
	float dx = cosf(pitchRad) * sinf(yawRad);
	float dz = cosf(pitchRad) * cosf(yawRad);
	float dy = 0;

	// sideways vector (perpendicular to forward in XZ)
	float sideX = cosf(yawRad);
	float sideZ = -sinf(yawRad);

	// compute lateral velocity from roll (banking effect)
	float rollInfluence = sinf(rollRad);
	float lateralSpeed = -rollInfluence * drone.maxSpeed * 0.8;
	// ^ adjust 0.5f scaling factor for strength of side-slip

	// update position (forward + vertical + lateral)
	drone.pos[0] += (dx * drone.speed + sideX * lateralSpeed) * deltaTime;
	drone.pos[1] += drone.vSpeed * deltaTime + dy * drone.speed * deltaTime;
	drone.pos[2] += (dz * drone.speed + sideZ * lateralSpeed) * deltaTime;

	// prevent drone from going below the floor
	if (drone.pos[1] < 1.0f) drone.pos[1] = 1.0f;

	/* --- Collision Handling (unchanged) --- */
	float dMin[3], dMax[3];
	computeDroneAABB(drone, dMin, dMax);

	int hitI = -1, hitJ = -1;
	if (findCollidingBuilding(dMin, dMax, hitI, hitJ)) {
		drone.pos[0] = prevPos[0];
		drone.pos[1] = prevPos[1];
		drone.pos[2] = prevPos[2];
		drone.speed = 0.0f;
		drone.vSpeed = 0.0f;

		const float spacing = 7.2f;
		const float floorSize = 45.0f;
		float xOffset = -floorSize / 2.0f + spacing / 2.0f;
		float zOffset = -floorSize / 2.0f + spacing / 2.0f;

		float bx = xOffset + hitI * spacing + buildingOffset[hitI][hitJ][0];
		float bz = zOffset + hitJ * spacing + buildingOffset[hitI][hitJ][2];

		float dirx = (bx - prevPos[0]);
		float dirz = (bz - prevPos[2]);
		float len = sqrtf(dirx * dirx + dirz * dirz);
		if (len < 1e-4f) { dirx = 1.0f; dirz = 0.0f; len = 1.0f; }
		dirx /= len; dirz /= len;

		const float pushAmount = 0.15f;
		buildingOffset[hitI][hitJ][0] += dirx * pushAmount;
		buildingOffset[hitI][hitJ][2] += dirz * pushAmount;

		const float maxOffset = 1.0f;
		buildingOffset[hitI][hitJ][0] = std::max(-maxOffset, std::min(maxOffset, buildingOffset[hitI][hitJ][0]));
		buildingOffset[hitI][hitJ][2] = std::max(-maxOffset, std::min(maxOffset, buildingOffset[hitI][hitJ][2]));
	}

	for (auto& o : flyingObjects) {
		float oMin[3], oMax[3];
		computeFlyingObjectAABB(o, oMin, oMax);
		if (aabbIntersect(dMin, dMax, oMin, oMax)) {
			resetDrone();
			break;
		}
	}
}


void animate(float deltaTime) {
	// --- Yaw control (A/D) ---
	if (keys['a']) drone.dirAngle -= drone.turnRate * deltaTime;
	if (keys['d']) drone.dirAngle += drone.turnRate * deltaTime;

	// --- Throttle (W/S) ---
	if (keys['w']) drone.vSpeed = std::min(drone.vSpeed + 0.2f, drone.maxVSpeed);
	if (keys['s']) drone.vSpeed = std::max(drone.vSpeed - 0.2f, -drone.maxVSpeed);

	// --- Pitch (arrow up/down) ---
	if (specialKeys[GLUT_KEY_UP]) {
		drone.speed = std::min(drone.speed + 0.2f, drone.maxSpeed); // accelerate forward
	}
	if (specialKeys[GLUT_KEY_DOWN]) {
		drone.speed = std::max(drone.speed - 0.2f, -drone.maxSpeed * 0.5f); // allow some backward drift
	}
	// --- Attitude dynamics (pitch/roll) ---
	float pitchAccel = 0.0f;
	if (specialKeys[GLUT_KEY_UP])   pitchAccel += ANG_INPUT_ACCEL;
	if (specialKeys[GLUT_KEY_DOWN]) pitchAccel -= ANG_INPUT_ACCEL;

	float rollAccel = 0.0f;
	if (specialKeys[GLUT_KEY_RIGHT]) rollAccel += ANG_INPUT_ACCEL;
	if (specialKeys[GLUT_KEY_LEFT])  rollAccel -= ANG_INPUT_ACCEL;

	// Damped spring back to level (adds coast + auto-level)
	pitchAccel += -ANG_DAMP * drone.pitchVel - ANG_SPRING * drone.pitch;
	rollAccel += -ANG_DAMP * drone.rollVel - ANG_SPRING * drone.roll;

	// Integrate angular velocity and angle
	drone.pitchVel += pitchAccel * deltaTime;
	drone.rollVel += rollAccel * deltaTime;

	drone.pitch += drone.pitchVel * deltaTime;
	drone.roll += drone.rollVel * deltaTime;

	// Clamp angles and prevent pushing further past the limit
	if (drone.pitch > MAX_PITCH) { drone.pitch = MAX_PITCH; if (drone.pitchVel > 0) drone.pitchVel = 0; }
	if (drone.pitch < -MAX_PITCH) { drone.pitch = -MAX_PITCH; if (drone.pitchVel < 0) drone.pitchVel = 0; }
	if (drone.roll > MAX_ROLL) { drone.roll = MAX_ROLL;  if (drone.rollVel > 0) drone.rollVel = 0; }
	if (drone.roll < -MAX_ROLL) { drone.roll = -MAX_ROLL;  if (drone.rollVel < 0) drone.rollVel = 0; }

	// Mild linear speed damping so forward motion “glides” a bit
	drone.vSpeed *= 0.98f;
	drone.speed *= 0.995f;

	updateDrone(deltaTime);
}

float lastTime = 0.0f;

void renderSim(void) {

	FrameCount++;

	float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float deltaTime = currentTime - lastTime;
	lastTime = currentTime;
	animate(deltaTime);

	float distance = 20.0f; // how far behind
	float height = 10.0f;  // how far above
	// apply offsets from mouse
	float yawRad = (drone.dirAngle + cam2_yawOffset) * 3.14159f / 180.0f;
	float pitchRadC = cam2_pitchOffset * 3.14159f / 180.0f;

	cams[2].pos[0] = drone.pos[0] - sin(yawRad) * cos(pitchRadC) * distance;
	cams[2].pos[1] = drone.pos[1] + height + sin(pitchRadC) * distance;
	cams[2].pos[2] = drone.pos[2] - cos(yawRad) * cos(pitchRadC) * distance;

	cams[2].target[0] = drone.pos[0];
	cams[2].target[1] = drone.pos[1];
	cams[2].target[2] = drone.pos[2];

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderer.activateRenderMeshesShaderProg(); // use the required GLSL program to draw the meshes with illumination
	int depthFog = 1; // 0 = z-based, 1 = radial
	float fogColor[3] = { 0.5f, 0.5f, 0.6f }; // light gray-blue
	float fogDensity = 0.03f;

	if (fogEnabled) {
		int depthFog = 1; // 0 = z-based, 1 = radial
		float fogColor[3] = { 0.5f, 0.5f, 0.6f }; // light gray-blue
		float fogDensity = 0.04f;
		renderer.setFogParams(depthFog, fogColor, fogDensity);
	}
	else {
		int depthFog = 1; // 0 = z-based, 1 = radial
		float fogColor[3] = { 0.0f, 0.0f, 0.0f }; // light gray-blue
		float fogDensity = 0.0f;
		renderer.setFogParams(depthFog, fogColor, fogDensity);
	}

	//Associar os Texture Units aos Objects Texture
	//stone.tga loaded in TU0; checker.tga loaded in TU1;  lightwood.tga loaded in TU2
	renderer.setTexUnit(0, 0);
	renderer.setTexUnit(1, 1);
	renderer.setTexUnit(2, 2);
	renderer.setTexUnit(3, 3);
	renderer.setTexUnit(4, 4);
	renderer.setTexUnit(5, 5);
	renderer.setTexUnit(6, 6);
	renderer.setTexUnit(7, 7);
	renderer.setTexUnit(8, 8);

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

	/*sending lights to the renderer*/

	// Directional
	float dirLightAux[4];
	mu.multMatrixPoint(gmu::VIEW, directionalLightPos, dirLightAux);
	renderer.setDirectionalLight(dirLightAux, 1.0f, 1.0f, 1.0f, directionalLightOn);

	// Point lights
	for (int i = 0; i < NUMBER_POINT_LIGHTS; i++) {
		mu.multMatrixPoint(gmu::VIEW, pointLightPos[i], pointLightEye[i]);
		pointLightEye[i][0] /= pointLightEye[i][3];
		pointLightEye[i][1] /= pointLightEye[i][3];
		pointLightEye[i][2] /= pointLightEye[i][3];
	}
	renderer.setPointLights(pointLightEye, pointLightColor, pointLightsOn);

	// Spotlights
	
	float yawRadians = drone.dirAngle * 3.14159f / 180.0f;
	float pitchRad = drone.pitch * 3.14159f / 180.0f;

	float fwd[3] = {
	cosf(pitchRad) * sinf(yawRadians),
	sinf(pitchRad),
	cosf(pitchRad) * cosf(yawRadians)
	};

	float right[3] = { cosf(yawRadians), 0.0f, -sinf(yawRadians) }; // perp to forward
	const float lateralOffset = 2.0f;  // left/right separation
	const float verticalOffset = -0.5f;  // a bit below the center
	float spotWorldPos[NUMBER_SPOT_LIGHTS][4];
	float spotWorldDir[NUMBER_SPOT_LIGHTS][4];

	// left
	spotWorldPos[0][0] = drone.pos[0] - right[0] * lateralOffset;
	spotWorldPos[0][1] = drone.pos[1] + verticalOffset;
	spotWorldPos[0][2] = drone.pos[2] - right[2] * lateralOffset;
	spotWorldPos[0][3] = 1.0f;
	spotWorldDir[0][0] = fwd[0];
	spotWorldDir[0][1] = fwd[1];
	spotWorldDir[0][2] = fwd[2];
	spotWorldDir[0][3] = 0.0f;

	// right
	spotWorldPos[1][0] = drone.pos[0] + right[0] * lateralOffset;
	spotWorldPos[1][1] = drone.pos[1] + verticalOffset;
	spotWorldPos[1][2] = drone.pos[2] + right[2] * lateralOffset;
	spotWorldPos[1][3] = 1.0f;
	spotWorldDir[1][0] = fwd[0];
	spotWorldDir[1][1] = fwd[1];
	spotWorldDir[1][2] = fwd[2];
	spotWorldDir[1][3] = 0.0f;

	float spotEyePos[NUMBER_SPOT_LIGHTS][4];
	float spotEyeDir[NUMBER_SPOT_LIGHTS][4];

	for (int i = 0; i < NUMBER_SPOT_LIGHTS; ++i) {
		// transform position (as a point: w=1)
		mu.multMatrixPoint(gmu::VIEW, spotWorldPos[i], spotEyePos[i]);

		// transform direction (as a vector: w=0)
		float dir4[4] = { spotWorldDir[i][0], spotWorldDir[i][1], spotWorldDir[i][2], 0.0f };
		mu.multMatrixPoint(gmu::VIEW, dir4, spotEyeDir[i]);

		// normalize result
		float len = sqrtf(spotEyeDir[i][0] * spotEyeDir[i][0] +
			spotEyeDir[i][1] * spotEyeDir[i][1] +
			spotEyeDir[i][2] * spotEyeDir[i][2]);
		if (len > 1e-6f) {
			spotEyeDir[i][0] /= len;
			spotEyeDir[i][1] /= len;
			spotEyeDir[i][2] /= len;
		}
	}

	float spotColor[NUMBER_SPOT_LIGHTS][3] = {
	{3.0f, 3.0f, 3.0f},
	{3.0f, 3.0f, 3.0f}
	};

	renderer.setSpotLights(spotEyePos, spotEyeDir, spotColor, spotLightsOn, spotCutOff);

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
	data.texMode = 6; //two texels blended
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
			int mID = city[i][j].meshID;
			if (mID == glassCubeMeshID || mID == glassCylMeshID) continue; // skip glass here

			mu.pushMatrix(gmu::MODEL);
			// same translate as before:
			mu.translate(gmu::MODEL,
				xOffset + (float)i * spacing + buildingOffset[i][j][0],
				0.0f + buildingOffset[i][j][1],
				zOffset + (float)j * spacing + buildingOffset[i][j][2]);

			float height = city[i][j].height;

			// same cube/cylinder transform (handle cylinder lift)
			if (mID == 3) { // cylinder original index
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
				mu.translate(gmu::MODEL, 0.0f, 0.5f, 0.0f);
			}
			else {
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
			}

			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();

			data.meshID = mID;
			data.texMode = city[i][j].texMode;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);

			mu.popMatrix(gmu::MODEL);
		}
	}
	// ----- TRANSPARENT BUILDINGS -----
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE); // keep depth test ON, but don't write depth

	for (int i = 0; i < GRID_SIZE; ++i) {
		for (int j = 0; j < GRID_SIZE; ++j) {
			int mID = city[i][j].meshID;
			if (mID != glassCubeMeshID && mID != glassCylMeshID) continue; // only glass here

			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL,
				xOffset + (float)i * spacing + buildingOffset[i][j][0],
				0.0f + buildingOffset[i][j][1],
				zOffset + (float)j * spacing + buildingOffset[i][j][2]);

			float height = city[i][j].height;

			// same transforms; handle the glass cylinder like the original cylinder
			if (mID == glassCylMeshID) {
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
				mu.translate(gmu::MODEL, 0.0f, 0.5f, 0.0f);
			}
			else {
				mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
			}

			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();

			data.meshID = mID;
			data.texMode = city[i][j].texMode; // any texMode still works; alpha comes from mat.a (and tex a if PNG/TGA)
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);

			mu.popMatrix(gmu::MODEL);
		}
	}

	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);

	// drone rendering
	/*float droneScale = 1.0f;

	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
	mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f);  // yaw rotation
	mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f);     // pitch rotation
	mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f);      // roll rotation
	mu.scale(gmu::MODEL, droneScale, droneScale, droneScale);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	data.meshID = 1;  // cube mesh, for now
	data.texMode = 0; //no texturing
	data.vm = mu.get(gmu::VIEW_MODEL),
	data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	data.normal = mu.getNormalMatrix();
	renderer.renderMesh(data);
	mu.popMatrix(gmu::MODEL);*/
	// DRONE RENDERING
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
	mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f);
	mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f);
	mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f);

	// --- Body ---
	// The repo's cube is in [0,1]^3 with origin at the min corner.
	// Center it at (0,0,0) first, THEN scale, so the drone origin is the body center.
	float bodyScaleX = 2.0f;
	float bodyScaleY = 0.6f;
	float bodyScaleZ = 2.0f;

	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f);  // center unit cube around origin
	mu.scale(gmu::MODEL, bodyScaleX, bodyScaleY, bodyScaleZ);
	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();
	data.meshID = droneBodyMeshID;
	data.texMode = 0;
	data.vm = mu.get(gmu::VIEW_MODEL);
	data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	data.normal = mu.getNormalMatrix();
	renderer.renderMesh(data);
	mu.popMatrix(gmu::MODEL);

	// --- Rotors (4 corners around the now-correct body center) ---
	float rotorMargin = 0.0f;  // set to 0
	float rotorHeight = 0.2f;

	// half extents of the body (since it's centered at origin now)
	float halfX = bodyScaleX * 0.5f + rotorMargin;
	float halfZ = bodyScaleZ * 0.5f + rotorMargin;

	for (int ix = -1; ix <= 1; ix += 2) {
		for (int iz = -1; iz <= 1; iz += 2) {
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, ix * halfX, rotorHeight, iz * halfZ);

			// rotor mesh (cylinder) is already centered in XZ in this project,
			// and height is in [-0.5,0.5] before scaling, so no centering translate needed.
			mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
			mu.scale(gmu::MODEL, 1.7f, 0.3f, 1.7f);

			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = droneRotorMeshID;
			data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);
		}
	}

	mu.popMatrix(gmu::MODEL);

	

	for (const auto& o : flyingObjects) {
		

		mu.pushMatrix(gmu::MODEL);
		mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
		if (o.rotAxis == 0) mu.rotate(gmu::MODEL, o.rotAngle, 1.0f, 0.0f, 0.0f);
		else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 1.0f, 0.0f);
		else mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 0.0f, 1.0f);

		mu.scale(gmu::MODEL, o.size, o.size, o.size);

		mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
		mu.computeNormalMatrix3x3();

		data.meshID = o.meshID;
		data.texMode = 0;
		data.vm = mu.get(gmu::VIEW_MODEL);
		data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
		data.normal = mu.getNormalMatrix();

		
		renderer.renderMesh(data);
		mu.popMatrix(gmu::MODEL);

	}
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
void keyboardDown(unsigned char key, int x, int y) {
	keys[key] = true;

	switch (key) {
	case 27:
		glutLeaveMainLoop();
		break;

	case '1': activeCam = 0; break;
	case '2': activeCam = 1; break;
	case '3': activeCam = 2; break;

	case 'c': pointLightsOn = !pointLightsOn; break;
	case 'h': spotLightsOn = !spotLightsOn; break;

	/*case 'l':
		spotlight_mode = !spotlight_mode;
		printf(spotlight_mode ? "Point light disabled. Spot light enabled\n"
			: "Spot light disabled. Point light enabled\n");
		break;*/

	case 'r':
		alpha = 57.0f; beta = 18.0f; r = 45.0f;
		camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
		camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
		camY = r * sin(beta * 3.14f / 180.0f);
		break;

	case 'm': glEnable(GL_MULTISAMPLE); break;
	case 'n': directionalLightOn = !directionalLightOn; break;
	case 'f': fogEnabled = !fogEnabled; break;

	case 'b': // toggle wire AABB
		debugDrawFlyingAABB = !debugDrawFlyingAABB;
		break;
	case '[': // prev flying object
		if (NUM_FLYING_OBJECTS > 0) {
			debugFlyIndex = (debugFlyIndex - 1 + NUM_FLYING_OBJECTS) % NUM_FLYING_OBJECTS;
		}
		break;
	case ']': // next flying object
		if (NUM_FLYING_OBJECTS > 0) {
			debugFlyIndex = (debugFlyIndex + 1) % NUM_FLYING_OBJECTS;
		}
		break;

	}
}
void keyboardUp(unsigned char key, int x, int y) {
	keys[key] = false;
}

void specialDown(int key, int x, int y) {
	specialKeys[key] = true;
}

void specialUp(int key, int x, int y) {
	specialKeys[key] = false;
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
	int deltaX = xx - startX;
	int deltaY = yy - startY;

	if (tracking == 1) {
		if (activeCam == 2) {
			// --- NEW: mouse orbiting for chase cam ---
			cam2_yawOffset += deltaX * 0.2f;   // sensitivity
			cam2_pitchOffset += -deltaY * 0.2f;

			// clamp pitch so camera doesn’t flip
			if (cam2_pitchOffset > 60.0f)  cam2_pitchOffset = 60.0f;
			if (cam2_pitchOffset < -30.0f) cam2_pitchOffset = -30.0f;

			startX = xx;
			startY = yy;
		}
		else {
			// --- OLD: free orbit cameras (0 or 1) ---
			float alphaAux = alpha - (xx - startX);
			float betaAux = beta + (yy - startY);
			if (betaAux > 85.0f) betaAux = 85.0f;
			if (betaAux < -85.0f) betaAux = -85.0f;

			float rAux = r;
			float camXtemp = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
			float camZtemp = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
			float camYtemp = rAux * sin(betaAux * 3.14f / 180.0f);

			camX = camXtemp;
			camY = camYtemp;
			camZ = camZtemp;

			cams[activeCam].pos[0] = camX;
			cams[activeCam].pos[1] = camY;
			cams[activeCam].pos[2] = camZ;

			cams[activeCam].target[0] = 0.0f;
			cams[activeCam].target[1] = 0.0f;
			cams[activeCam].target[2] = 0.0f;
		}
	}
	// right mouse button zoom (unchanged)
	else if (tracking == 2) {
		r += (yy - startY) * 0.01f;
		if (r < 0.1f) r = 0.1f;
	}
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
	renderer.TexObjArray.texture2D_Loader("assets/concrete.jpg");
	renderer.TexObjArray.texture2D_Loader("assets/cobblestone.png");
	renderer.TexObjArray.texture2D_Loader("assets/skyscraper_night.jpg");
	renderer.TexObjArray.texture2D_Loader("assets/skyscraper_plain.jpg");
	renderer.TexObjArray.texture2D_Loader("assets/skyscraper_glass.jpg");
	renderer.TexObjArray.texture2D_Loader("assets/skyscraper_residential.jpg");

	//Scene geometry with triangle meshes

	MyMesh amesh;

	float amb[] = { 0.2f, 0.15f, 0.1f, 1.0f };
	float diff[] = { 0.8f, 0.6f, 0.4f, 1.0f };
	float spec[] = { 0.8f, 0.8f, 0.8f, 1.0f };

	float amb1[] = { 0.3f, 0.0f, 0.0f, 1.0f };
	float diff1[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	float spec1[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	float emissive[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float shininess = 100.0f;
	int texcount = 0;

	float ambNeutral[] = { 0.20f, 0.20f, 0.20f, 1.0f };

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
	memcpy(amesh.mat.ambient, ambNeutral, 4 * sizeof(float));
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
	memcpy(amesh.mat.ambient, ambNeutral, 4 * sizeof(float));
	memcpy(amesh.mat.diffuse, diff, 4 * sizeof(float));
	memcpy(amesh.mat.specular, spec, 4 * sizeof(float));
	memcpy(amesh.mat.emissive, emissive, 4 * sizeof(float));
	amesh.mat.shininess = shininess;
	amesh.mat.texCount = texcount;
	renderer.myMeshes.push_back(amesh);
	{
		// Glassy cube
		MyMesh glass = createCube();
		float amb[] = { 0.05f, 0.08f, 0.10f, 0.6f };   // a ~ 0.55
		float diff[] = { 0.20f, 0.45f, 0.65f, 0.6f };
		float spec[] = { 0.80f, 0.90f, 1.00f, 0.6f };
		float emis[] = { 0.00f, 0.00f, 0.00f, 0.6f };
		glass.mat.shininess = 120.0f; glass.mat.texCount = 0;
		memcpy(glass.mat.ambient, amb, 4 * sizeof(float));
		memcpy(glass.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(glass.mat.specular, spec, 4 * sizeof(float));
		memcpy(glass.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(glass);
		glassCubeMeshID = (int)renderer.myMeshes.size() - 1;
	}

	{

		// Glassy cylinder
		MyMesh glass = createCylinder(1.5f, 0.5f, 20);
		float amb[] = { 0.05f, 0.08f, 0.10f, 0.6f };
		float diff[] = { 0.20f, 0.45f, 0.65f, 0.6f };
		float spec[] = { 0.80f, 0.90f, 1.00f, 0.6f };
		float emis[] = { 0.00f, 0.00f, 0.00f, 0.6f };
		glass.mat.shininess = 120.0f; glass.mat.texCount = 0;
		memcpy(glass.mat.ambient, amb, 4 * sizeof(float));
		memcpy(glass.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(glass.mat.specular, spec, 4 * sizeof(float));
		memcpy(glass.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(glass);
		glassCylMeshID = (int)renderer.myMeshes.size() - 1;

	}

	

	// -- - Drone body(flattened cube) -- -
	{
		MyMesh body = createCube();
		float amb[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		float diff[] = { 0.6f, 0.6f, 0.6f, 1.0f };
		float spec[] = { 0.8f, 0.8f, 0.8f, 1.0f };
		float emis[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		body.mat.shininess = 80.0f; body.mat.texCount = 0;
		memcpy(body.mat.ambient, amb, 4 * sizeof(float));
		memcpy(body.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(body.mat.specular, spec, 4 * sizeof(float));
		memcpy(body.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(body);
		droneBodyMeshID = (int)renderer.myMeshes.size() - 1;
	}

	// --- Drone rotor (cylinder) ---
	{
		MyMesh rotor = createCylinder(1.0f, 0.2f, 20); // wide & flat
		float amb[] = { 0.1f, 0.1f, 0.1f, 1.0f };
		float diff[] = { 0.4f, 0.4f, 0.4f, 1.0f };
		float spec[] = { 0.8f, 0.8f, 0.8f, 1.0f };
		float emis[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		rotor.mat.shininess = 100.0f; rotor.mat.texCount = 0;
		memcpy(rotor.mat.ambient, amb, 4 * sizeof(float));
		memcpy(rotor.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(rotor.mat.specular, spec, 4 * sizeof(float));
		memcpy(rotor.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(rotor);
		droneRotorMeshID = (int)renderer.myMeshes.size() - 1;
	}

	{
		MyMesh wire = createCube();  // unit cube in [0,1]^3

		// Bright green material (no texture)
		float amb[] = { 0.0f, 0.25f, 0.0f, 1.0f };
		float dif[] = { 0.0f, 0.9f,  0.0f, 1.0f };
		float spec[] = { 0.1f, 0.1f, 0.1f, 1.0f };
		float emis[] = { 0.0f, 0.0f,  0.0f, 1.0f };
		wire.mat.shininess = 10.0f; wire.mat.texCount = 0;
		memcpy(wire.mat.ambient, amb, 4 * sizeof(float));
		memcpy(wire.mat.diffuse, dif, 4 * sizeof(float));
		memcpy(wire.mat.specular, spec, 4 * sizeof(float));
		memcpy(wire.mat.emissive, emis, 4 * sizeof(float));

		renderer.myMeshes.push_back(wire);
		wireCubeMeshID = (int)renderer.myMeshes.size() - 1;
	}

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

	srand((unsigned)time(NULL));

	// Dedicated small sphere for flying objects (bright so it’s visible)
	{
		MyMesh mob = createCube();
		float amb[] = { 0.1f, 0.1f, 0.1f, 1.0f };
		float diff[] = { 0.8f, 0.8f, 0.2f, 1.0f };
		float spec[] = { 0.9f, 0.9f, 0.9f, 1.0f };
		float emis[] = { 0.05f, 0.05f, 0.05f, 1.0f };
		mob.mat.shininess = 80.0f; mob.mat.texCount = 0;
		memcpy(mob.mat.ambient, amb, 4 * sizeof(float));
		memcpy(mob.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(mob.mat.specular, spec, 4 * sizeof(float));
		memcpy(mob.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(mob);
		meshFlyingObject = (int)renderer.myMeshes.size() - 1;
	}

	// Create the initial list
	initFlyingObjects();
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
	glutTimerFunc(0, refresh, 0);    //use it to to get 60 FPS whatever

//	Mouse and Keyboard Callbacks
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutSpecialFunc(specialDown);
	glutSpecialUpFunc(specialUp);
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



