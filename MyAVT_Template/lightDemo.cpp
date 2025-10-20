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
#include "flare.h" 

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
long myTime, timebase = 0, frame = 0;
char s[32];

bool fontLoaded = false;

// fog flag
bool fogEnabled = true;

float aspectRatio = 1.0f;

bool debugDrawFlyingAABB = false;  // toggle with 'b'
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

//pause flag 
bool paused = false;

struct Camera {
	float pos[3] = { 0.0f, 0.0f, 0.0f };
	float target[3] = { 0.0f, 0.0f, 0.0f };
	int type = 0; // 0 = perspective, 1 = orthographic
};
Camera cams[3];
int activeCam = 0;

int   stencilMaskMeshID = -1;
bool  showRearCam = true;   // toggle if you want
float rearMaskSizePx = 220.0f; // diamond size in screen pixels
float rearMaskMarginPx = 20.0f;  // margin from the window edges

struct Drone {
	float pos[3] = { -30.0f, 15.0f, 0.0f }; // world position (x,y,z)
	float dirAngle = 90.0f;   // yaw (degrees) — heading in the XZ plane
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

	bool hasStarted = false;
};

Drone drone;
float batteryLevel = 100.0f;      // 100.0 = full, 0.0 = empty
int playerScore = 0;
bool hasPackage = false;
bool gameOver = false;

// battery parameters
const float BATTERY_DRAIN_RATE = 2.0f;  // proportional to throttle
const float COLLISION_PENALTY = 20.0f;   // lose 20% battery per crash

float lastCollisionTime = -1.0f;
const float collisionCooldown = 1.0f; // 1 second delay between two collision penalties

// HUD GL objects
GLuint hudVAO = 0, hudVBO = 0, hudProgram = 0;

bool gPrintBillboardCoords = false;  // set false to stop printing


struct Package {
	int i, j;        // which building it's on (grid indices)
	float pos[3];    // world position (x, y, z)
	bool active;     // true if visible

	bool beingCarried = false;
	bool delivered = false;
	int destI, destJ;
	float destPos[3];
};

Package package;
int packageMeshID = -1;

struct SmokeParticle {
	float pos[3];
	float vel[3];
	float life;
	float maxLife;
	float size;
};

std::vector<SmokeParticle> g_smoke;
int smokeQuadMeshID = -1;
//GLuint smokeTextureID = 0;

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

const int GRID_SIZE = 9;
Building city[GRID_SIZE][GRID_SIZE];
float buildingOffset[GRID_SIZE][GRID_SIZE][3];
int beamMeshID = -1;

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

const int   NUM_FLYING_OBJECTS = 1;
const float SPAWN_RADIUS = 45.0f;  // birth ring radius (XZ)
const float KILL_RADIUS = 65.0f;  // die when too far from center
const float MIN_Y = 0.0f;
const float MAX_Y = 20.0f;

const float MAX_XZ = 30.0f;
const float MIN_XZ = -30.0f;


const float BASE_SPEED = 20.0f;   // initial median speed

// Difficulty ramp (~every 30 s)
float speedFactor = 1.0f;
float nextSpeedBump = 5.0f;  // seconds


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

static GLuint gSkyboxTex = 0;         // GL cubemap
static int    skyboxCubeMeshID = -1;

// ----- Lens flare globals -----
bool flareEffect = false;      // toggled with 'l' (only when spotlights are OFF)
FLARE_DEF gFlare;              // parsed from flare.txt
GLuint FlareTextureArray[NTEXTURES] = { 0 };  // crcl, flar, hxgn, ring, sun
// we'll reuse the smoke billboard quad for flare elements:

float sunAzimuthDeg = -30.0f;  // around Y (0 = +Z)
float sunElevationDeg = 30;   // 0=horizon, 90=zenith
float sunDistance = 90.0f;  // how far from scene center

static void setSunFromAngles(float azDeg, float elDeg) {
	const float PI = 3.1415926f;
	float az = azDeg * PI / 180.0f;
	float el = elDeg * PI / 180.0f;

	// direction from sky to ground
	float dir[3] = {
		sinf(az) * cosf(el),
		-sinf(el),                 // negative Y = pointing down
		cosf(az) * cosf(el)
	};

	// use same dir for the directional light (w=0)
	directionalLightPos[0] = dir[0];
	directionalLightPos[1] = dir[1];
	directionalLightPos[2] = dir[2];
	directionalLightPos[3] = 0.0f;

	// place point light along that dir, far from the center (0,0,0)
	pointLightPos[0][0] = -dir[0] * sunDistance;
	pointLightPos[0][1] = -dir[1] * sunDistance;
	pointLightPos[0][2] = -dir[2] * sunDistance;
	pointLightPos[0][3] = 1.0f;

	// brighten it a bit since it’s far away (your shader supports >1.0 intensities)
	pointLightColor[0][0] = pointLightColor[0][1] = pointLightColor[0][2] = 3.0f;
}

static inline int flareQuadMeshID() { return smokeQuadMeshID; }

static int getTextureId(char* name) {
	for (int i = 0; i < NTEXTURES; ++i) {
		if (strncmp(name, flareTextureNames[i], (int)strlen(name)) == 0) return i;
	}
	return -1;
}

void loadFlareFile(FLARE_DEF* flare, char* /*filename*/) {
	int n = 0;
	FILE* f = fopen("assets/flare.txt", "r");
	if (!f) f = fopen("flare.txt", "r");
	memset(flare, 0, sizeof(FLARE_DEF));
	if (!f) { printf("Flare file opening error\n"); return; }

	char buf[256];
	fgets(buf, sizeof(buf), f);
	sscanf(buf, "%f %f", &flare->fScale, &flare->fMaxSize);

	while (!feof(f) && n < FLARE_MAXELEMENTSPERFLARE) {
		char  name[8] = { 0 };
		double dDist = 0.0, dSize = 0.0;
		float color[4]; // A R G B
		int fields = 0;
		fgets(buf, sizeof(buf), f);
		fields = sscanf(buf, "%4s %lf %lf ( %f %f %f %f )", name, &dDist, &dSize, &color[3], &color[0], &color[1], &color[2]);
		if (fields == 7) {
			for (int i = 0;i < 4;++i) color[i] = std::max(0.0f, std::min(1.0f, color[i] / 255.0f));
			int id = getTextureId(name);
			if (id < 0) printf("Flare texture name '%s' not recognized\n", name);
			else flare->element[n].textureId = id;

			flare->element[n].fDistance = (float)dDist;
			flare->element[n].fSize = (float)dSize;
			memcpy(flare->element[n].matDiffuse, color, 4 * sizeof(float));
			++n;
		}
	}
	flare->nPieces = n;
	fclose(f);
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

void updateSmokeParticles(float dt) {
	for (auto& p : g_smoke) {
		p.life -= dt;
		if (p.life > 0.0f) {
			p.pos[0] += p.vel[0] * dt;
			p.pos[1] += p.vel[1] * dt;
			p.pos[2] += p.vel[2] * dt;
		}
	}
	// remove dead particles
	g_smoke.erase(
		std::remove_if(g_smoke.begin(), g_smoke.end(),
			[](const SmokeParticle& p) { return p.life <= 0.0f; }),
		g_smoke.end());
}


void refresh(int value)
{
	static float last = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float dt = now - last;
	last = now;

	if (!paused) {
		updateFlyingObjects(dt, now);
		updateSmokeParticles(dt);
	}

	glutPostRedisplay();
	glutTimerFunc(16, refresh, 0);
}

// Reshape Callback Function
void changeSize(int w, int h) {
	if (h == 0) h = 1;  // prevent divide by zero

	// set viewport
	glViewport(0, 0, w, h);

	// update aspect ratio for later use in renderSim
	aspectRatio = static_cast<float>(w) / static_cast<float>(h);
}

static inline void reflectY(float v[4]) {
	v[1] = -v[1];
}

static void setLightsForPass(bool reflected) {
	// Directional
	float dirL[4] = { directionalLightPos[0], directionalLightPos[1],
					  directionalLightPos[2], directionalLightPos[3] }; // w=0
	if (reflected) reflectY(dirL);
	float dirEye[4];
	mu.multMatrixPoint(gmu::VIEW, dirL, dirEye);
	renderer.setDirectionalLight(dirEye, 1.0f, 1.0f, 1.0f, directionalLightOn);

	// Point lights
	for (int i = 0;i < NUMBER_POINT_LIGHTS;++i) {
		float L[4] = { pointLightPos[i][0], pointLightPos[i][1],
					   pointLightPos[i][2], pointLightPos[i][3] }; // w=1
		if (reflected) reflectY(L);
		mu.multMatrixPoint(gmu::VIEW, L, pointLightEye[i]);
		pointLightEye[i][0] /= pointLightEye[i][3];
		pointLightEye[i][1] /= pointLightEye[i][3];
		pointLightEye[i][2] /= pointLightEye[i][3];
	}
	renderer.setPointLights(pointLightEye, pointLightColor, pointLightsOn);

	// Spotlights (from your drone)
	float yawRadians = drone.dirAngle * 3.14159f / 180.0f;
	float pitchRad = drone.pitch * 3.14159f / 180.0f;

	float fwd[3] = { cosf(pitchRad) * sinf(yawRadians),
					 sinf(pitchRad),
					 cosf(pitchRad) * cosf(yawRadians) };
	float right[3] = { cosf(yawRadians), 0.0f, -sinf(yawRadians) };
	const float lateralOffset = 2.0f;
	const float verticalOffset = -0.5f;

	float spotWorldPos[NUMBER_SPOT_LIGHTS][4];
	float spotWorldDir[NUMBER_SPOT_LIGHTS][4];

	// left
	spotWorldPos[0][0] = drone.pos[0] - right[0] * lateralOffset;
	spotWorldPos[0][1] = drone.pos[1] + verticalOffset;
	spotWorldPos[0][2] = drone.pos[2] - right[2] * lateralOffset;
	spotWorldPos[0][3] = 1.0f;
	spotWorldDir[0][0] = fwd[0]; spotWorldDir[0][1] = fwd[1]; spotWorldDir[0][2] = fwd[2]; spotWorldDir[0][3] = 0.0f;

	// right
	spotWorldPos[1][0] = drone.pos[0] + right[0] * lateralOffset;
	spotWorldPos[1][1] = drone.pos[1] + verticalOffset;
	spotWorldPos[1][2] = drone.pos[2] + right[2] * lateralOffset;
	spotWorldPos[1][3] = 1.0f;
	spotWorldDir[1][0] = fwd[0]; spotWorldDir[1][1] = fwd[1]; spotWorldDir[1][2] = fwd[2]; spotWorldDir[1][3] = 0.0f;

	if (reflected) {
		reflectY(spotWorldPos[0]); reflectY(spotWorldPos[1]);
		reflectY(spotWorldDir[0]); reflectY(spotWorldDir[1]);
	}

	float spotEyePos[NUMBER_SPOT_LIGHTS][4];
	float spotEyeDir[NUMBER_SPOT_LIGHTS][4];
	for (int i = 0;i < NUMBER_SPOT_LIGHTS;++i) {
		mu.multMatrixPoint(gmu::VIEW, spotWorldPos[i], spotEyePos[i]);
		float d4[4] = { spotWorldDir[i][0], spotWorldDir[i][1], spotWorldDir[i][2], 0.0f };
		mu.multMatrixPoint(gmu::VIEW, d4, spotEyeDir[i]);
		float len = sqrtf(spotEyeDir[i][0] * spotEyeDir[i][0] + spotEyeDir[i][1] * spotEyeDir[i][1] + spotEyeDir[i][2] * spotEyeDir[i][2]);
		if (len > 1e-6f) { spotEyeDir[i][0] /= len; spotEyeDir[i][1] /= len; spotEyeDir[i][2] /= len; }
	}

	float spotColor[NUMBER_SPOT_LIGHTS][3] = { {3,3,3},{3,3,3} };
	renderer.setSpotLights(spotEyePos, spotEyeDir, spotColor, spotLightsOn, spotCutOff);
}

static void drawFloor(int texMode = 6) {
	dataMesh data{};
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, 0.0f, 0.0f, 0.0f);
	mu.rotate(gmu::MODEL, -90.0f, 1.0f, 0.0f, 0.0f);
	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();
	data.meshID = 0;
	data.texMode = texMode;
	data.vm = mu.get(gmu::VIEW_MODEL);
	data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	data.normal = mu.getNormalMatrix();
	renderer.renderMesh(data);
	mu.popMatrix(gmu::MODEL);
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
				city[i][j].meshID = (city[i][j].meshID == 3 ? glassCubeMeshID : glassCylMeshID);
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
	const float floorSize = 65.0f;
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
		outMin[0] = baseX - scaleX * 0.5f;
		outMax[0] = baseX + scaleX * 0.5f;
		outMin[2] = baseZ - scaleZ * 0.5f;
		outMax[2] = baseZ + scaleZ * 0.5f;
		outMin[1] = yOff;
		outMax[1] = yOff + 1.5f * h;   // full height above
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

void spawnPackage() {
	// Pick random source building
	package.i = rand() % GRID_SIZE;
	package.j = rand() % GRID_SIZE;

	float bMin[3], bMax[3];
	computeBuildingAABB(package.i, package.j, bMin, bMax);
	package.pos[0] = (bMin[0] + bMax[0]) / 2.0f;
	package.pos[1] = bMax[1] + 0.3f;
	package.pos[2] = (bMin[2] + bMax[2]) / 2.0f;
	package.active = true;
	package.beingCarried = false;
	package.delivered = false;

	// Pick a different building for delivery
	do {
		package.destI = rand() % GRID_SIZE;
		package.destJ = rand() % GRID_SIZE;
	} while (package.destI == package.i && package.destJ == package.j);

	computeBuildingAABB(package.destI, package.destJ, bMin, bMax);
	package.destPos[0] = (bMin[0] + bMax[0]) / 2.0f;
	package.destPos[1] = bMax[1] + 0.3f;
	package.destPos[2] = (bMin[2] + bMax[2]) / 2.0f;
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

	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);

	// base drone transform (translate + yaw + pitch + roll) — same order as render
	mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
	mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f); // yaw
	mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f); // pitch
	mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f); // roll

	// --- Body (matches renderSim) --
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f); // center cube
	mu.scale(gmu::MODEL, bodyScaleX, bodyScaleY, bodyScaleZ);
	accumulateCurrentMuModel(cubeMin, cubeMax);
	mu.popMatrix(gmu::MODEL);

	// -- Rotors (4 corners; matches renderSim order) --
	for (int ix = -1; ix <= 1; ix += 2) {
		for (int iz = -1; iz <= 1; iz += 2) {
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, ix * halfX, rotorHeight, iz * halfZ);
			mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
			mu.scale(gmu::MODEL, rotorScaleX, rotorScaleY, rotorScaleZ);
			accumulateCurrentMuModel(cylMin, cylMax);
			mu.popMatrix(gmu::MODEL);
		}
	}

	mu.popMatrix(gmu::MODEL);
}


void computeFlyingObjectAABB(const FlyingObject& o, float outMin[3], float outMax[3]) {

	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);

	mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
	if (o.rotAxis == 0)      mu.rotate(gmu::MODEL, o.rotAngle, 1.0f, 0.0f, 0.0f);
	else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 1.0f, 0.0f);
	else                     mu.rotate(gmu::MODEL, o.rotAngle, 0.0f, 0.0f, 1.0f);
	mu.scale(gmu::MODEL, o.size, o.size, o.size);

	Mat4 M = mat4FromGmu(mu.get(gmu::MODEL));
	mu.popMatrix(gmu::MODEL);

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
	drone.pos[0] = -30.0f;
	drone.pos[1] = 15.0f;
	drone.pos[2] = 0.0f;
	drone.dirAngle = 90.0f;
	drone.pitch = 0.0f;
	drone.roll = 0.0f;
	drone.speed = 0.0f;
	drone.vSpeed = 0.0f;
}

static inline void closestPointOnAABB(const float p[3], const float bmin[3], const float bmax[3], float out[3]) {
	out[0] = std::max(bmin[0], std::min(p[0], bmax[0]));
	out[1] = std::max(bmin[1], std::min(p[1], bmax[1]));
	out[2] = std::max(bmin[2], std::min(p[2], bmax[2]));
}

// Compute the 4 rotor centers in WORLD space (same layout/params as renderSim)
static void computeRotorWorldPositions(const Drone& d, float out[4][3]) {
	const float bodyScaleX = 2.0f;
	const float bodyScaleY = 0.6f; (void)bodyScaleY; // not needed here
	const float bodyScaleZ = 2.0f;

	const float rotorMargin = 0.0f;
	const float rotorHeight = 0.2f;
	const float halfX = bodyScaleX * 0.5f + rotorMargin;
	const float halfZ = bodyScaleZ * 0.5f + rotorMargin;

	// local rotor centers in DRONE MODEL space match the render order:
	// translate(ix*halfX, rotorHeight, iz*halfZ); translate(0.5,0,0.5);
	const float local[4][3] = {
		{ -halfX + 0.5f, rotorHeight, -halfZ + 0.5f }, // (-x,-z)
		{ -halfX + 0.5f, rotorHeight,  halfZ + 0.5f }, // (-x,+z)
		{  halfX + 0.5f, rotorHeight, -halfZ + 0.5f }, // (+x,-z)
		{  halfX + 0.5f, rotorHeight,  halfZ + 0.5f }, // (+x,+z)
	};

	// Build the same base transform used for the drone in renderSim (T * R_y * R_x * R_z)
	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);
	mu.translate(gmu::MODEL, d.pos[0], d.pos[1], d.pos[2]);
	mu.rotate(gmu::MODEL, d.dirAngle, 0.0f, 1.0f, 0.0f); // yaw
	mu.rotate(gmu::MODEL, d.pitch, 1.0f, 0.0f, 0.0f); // pitch
	mu.rotate(gmu::MODEL, d.roll, 0.0f, 0.0f, 1.0f); // roll

	Mat4 M = mat4FromGmu(mu.get(gmu::MODEL));
	mu.popMatrix(gmu::MODEL);

	for (int i = 0; i < 4; ++i) xformPoint(M, local[i], out[i]);
}


static inline void emitSmokeBurstAt(const float src[3], const float n[3], int count = 15) {
	// normalized normal (fallback up)
	float nx = n[0], ny = n[1], nz = n[2];
	float nlen = sqrtf(nx * nx + ny * ny + nz * nz);
	if (nlen > 1e-4f) { nx /= nlen; ny /= nlen; nz /= nlen; }
	else { nx = 0; ny = 1; nz = 0; }

	for (int i = 0; i < count; ++i) {
		SmokeParticle p{};
		p.pos[0] = src[0]; p.pos[1] = src[1]; p.pos[2] = src[2];

		// upward drift + slight push away from the surface
		p.vel[0] = frand(-0.3f, 0.3f) + nx * frand(0.2f, 0.6f);
		p.vel[1] = frand(1.0f, 2.2f) + ny * frand(0.1f, 0.4f);
		p.vel[2] = frand(-0.3f, 0.3f) + nz * frand(0.2f, 0.6f);

		p.life = p.maxLife = frand(1.0f, 2.0f);
		p.size = frand(0.8f, 1.5f);
		g_smoke.push_back(p);
	}
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
	const float floorY = 0.0f;
	const float droneHalfHeight = 0.5f;
	if (drone.pos[1] - droneHalfHeight < floorY)
		drone.pos[1] = floorY + droneHalfHeight;

	if (!drone.hasStarted && (std::abs(drone.speed) > 0.1f || std::abs(drone.vSpeed) > 0.1f)) {
		drone.hasStarted = true;
	}

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

		float now = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
		if (lastCollisionTime < 0 || now - lastCollisionTime > collisionCooldown) {
			batteryLevel -= COLLISION_PENALTY;
			batteryLevel = std::max(0.0f, batteryLevel);
			lastCollisionTime = now;

			float bMin[3], bMax[3];
			computeBuildingAABB(hitI, hitJ, bMin, bMax);

			float rotors[4][3];
			computeRotorWorldPositions(drone, rotors);

			// compute squared distance from each rotor to the building (via closest point)
			float d2Arr[4];
			float cp[4][3]; // closest points on the AABB to each rotor
			for (int i = 0; i < 4; ++i) {
				closestPointOnAABB(rotors[i], bMin, bMax, cp[i]);
				float dx = rotors[i][0] - cp[i][0];
				float dy = rotors[i][1] - cp[i][1];
				float dz = rotors[i][2] - cp[i][2];
				d2Arr[i] = dx * dx + dy * dy + dz * dz;
			}

			// find best and second-best indices
			int best = 0, second = 1;
			if (d2Arr[second] < d2Arr[best]) std::swap(best, second);
			for (int i = 2; i < 4; ++i) {
				if (d2Arr[i] < d2Arr[best]) { second = best; best = i; }
				else if (d2Arr[i] < d2Arr[second]) { second = i; }
			}

			// convert to actual distances for a nicer “tie” test
			float d1 = sqrtf(d2Arr[best]);
			float d2 = sqrtf(d2Arr[second]);

			// tie thresholds (tweak to taste)
			const float ROTOR_TIE_ABS_EPS = 0.25f;   // meters
			const float ROTOR_TIE_REL_EPS = 0.20f;   // within 20%

			bool emitTwo = ((fabsf(d2 - d1) < ROTOR_TIE_ABS_EPS) || (d1 > 1e-4f && (d2 / d1 - 1.0f) < ROTOR_TIE_REL_EPS));

			// normals (from wall-contact point toward rotor)
			float nBest[3] = { rotors[best][0] - cp[best][0],   rotors[best][1] - cp[best][1],   rotors[best][2] - cp[best][2] };
			float nSecond[3] = { rotors[second][0] - cp[second][0], rotors[second][1] - cp[second][1], rotors[second][2] - cp[second][2] };

			// emit
			emitSmokeBurstAt(rotors[best], nBest, 15);
			if (emitTwo) emitSmokeBurstAt(rotors[second], nSecond, 12); // slightly fewer for balance
		}

		const float spacing = 7.2f;
		const float floorSize = 65.0f;
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

	if (!gameOver && drone.hasStarted) {
		float throttle = std::abs(drone.vSpeed / drone.maxVSpeed);
		const float TIME_DRAIN_RATE = 0.3f;

		batteryLevel -= (BATTERY_DRAIN_RATE * throttle + TIME_DRAIN_RATE) * deltaTime;
		batteryLevel = std::max(0.0f, batteryLevel);

		if (batteryLevel <= 0.0f) {
			gameOver = true;
			drone.speed = 0.0f;
			drone.vSpeed = -3.0f;  // start falling
		}
	}

	// fall when out of battery
	if (gameOver && drone.pos[1] > 0.0f) {
		drone.vSpeed -= 9.8f * deltaTime;  // gravity
		drone.pos[1] += drone.vSpeed * deltaTime;
		if (drone.pos[1] <= 0.5f) {
			drone.pos[1] = 0.5f;
			drone.vSpeed = 0.0f;
		}
	}

	// --- PACKAGE PICKUP & DELIVERY ---
	if (package.active) {
		float dx = drone.pos[0] - package.pos[0];
		float dy = drone.pos[1] - package.pos[1];
		float dz = drone.pos[2] - package.pos[2];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		// Pickup if close enough
		if (!package.beingCarried && dist < 2.0f && fabs(dy) < 3.0f) {
			package.beingCarried = true;
		}

		// If carrying, attach to drone
		if (package.beingCarried) {
			package.pos[0] = drone.pos[0];
			package.pos[1] = drone.pos[1] - 1.5f;
			package.pos[2] = drone.pos[2];
		}

		// Drop off if near destination
		if (package.beingCarried) {
			float dx2 = drone.pos[0] - package.destPos[0];
			float dy2 = drone.pos[1] - package.destPos[1];
			float dz2 = drone.pos[2] - package.destPos[2];
			float dist2 = sqrtf(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

			if (dist2 < 2.0f && fabs(dy2) < 3.0f) {
				package.beingCarried = false;
				package.delivered = true;
				playerScore += (int)batteryLevel;  // score proportional to remaining battery
				batteryLevel = 100.0f;
				spawnPackage(); // start next delivery
			}
		}
	}

}


void animate(float deltaTime) {
	// --- Yaw control (A/D) ---
	if (keys['a']) drone.dirAngle += drone.turnRate * deltaTime;
	if (keys['d']) drone.dirAngle -= drone.turnRate * deltaTime;

	// --- Throttle (W/S) ---
	if (!gameOver) {
		if (keys['w']) drone.vSpeed = std::min(drone.vSpeed + 0.2f, drone.maxVSpeed);
		if (keys['s']) drone.vSpeed = std::max(drone.vSpeed - 0.2f, -drone.maxVSpeed);
	}
	else {
		// No manual thrust while falling; prevent any upward velocity
		if (drone.vSpeed > 0.0f) drone.vSpeed = 0.0f;
	}

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

void drawText2D(const std::string& msg, int x, int y, float scale,
	float r = 1.0f, float g = 1.0f, float b = 1.0f, float a = 1.0f) {
	if (!fontLoaded) return;

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	int vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);

	mu.loadIdentity(gmu::MODEL);
	mu.loadIdentity(gmu::VIEW);

	mu.pushMatrix(gmu::PROJECTION);
	mu.loadIdentity(gmu::PROJECTION);
	mu.ortho(vp[0], vp[0] + vp[2] - 1,
		vp[1], vp[1] + vp[3] - 1,
		-1, 1);
	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);

	TextCommand cmd;
	cmd.str = msg;
	cmd.position[0] = (float)x;
	cmd.position[1] = (float)y;
	cmd.size = scale;
	cmd.color[0] = r;
	cmd.color[1] = g;
	cmd.color[2] = b;
	cmd.color[3] = a;
	cmd.pvm = mu.get(gmu::PROJ_VIEW_MODEL);

	renderer.renderText(cmd);

	mu.popMatrix(gmu::PROJECTION);

	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}

void drawTextWithBackground(const std::string& msg, int x, int y, float scale,
	float textR, float textG, float textB, float textA,
	float bgR, float bgG, float bgB, float bgA,
	float paddingX = 10.0f, float paddingY = 5.0f)
{
	if (!fontLoaded) return;

	// --- estimate text size in pixels (same heuristic you used before) ---
	float textWidth = msg.size() * 20.0f * scale;  // tune if you have a better measurer
	float textHeight = 40.0f * scale;

	// background rectangle in screen-pixel coords
	float bgX = x - paddingX;
	float bgY = y - paddingY;
	float bgW = textWidth + 2.0f * paddingX;
	float bgH = textHeight + 2.0f * paddingY;

	// --- HUD state: depth off, blending on ---
	GLboolean depthWasOn = glIsEnabled(GL_DEPTH_TEST);
	GLboolean blendWasOn = glIsEnabled(GL_BLEND);
	if (depthWasOn) glDisable(GL_DEPTH_TEST);
	if (!blendWasOn) glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// --- set up 2D ortho in pixels ---
	int vp[4]; glGetIntegerv(GL_VIEWPORT, vp);
	mu.loadIdentity(gmu::MODEL);
	mu.loadIdentity(gmu::VIEW);

	mu.pushMatrix(gmu::PROJECTION);
	mu.loadIdentity(gmu::PROJECTION);
	mu.ortho((float)vp[0], (float)(vp[0] + vp[2] - 1),
		(float)vp[1], (float)(vp[1] + vp[3] - 1),
		-1.0f, 1.0f);

	// --- render a colored quad via the renderer (no glBegin) ---
	renderer.activateRenderMeshesShaderProg();

	// position quad at the center of the bg rect, then scale to its size
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, bgX + 0.5f * bgW, bgY + 0.5f * bgH, 0.0f);
	mu.scale(gmu::MODEL, bgW, bgH, 1.0f);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	dataMesh d{};
	d.meshID = (smokeQuadMeshID >= 0) ? smokeQuadMeshID : stencilMaskMeshID; // any unit 1x1 quad
	d.texMode = 0; // flat color
	d.vm = mu.get(gmu::VIEW_MODEL);
	d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	d.normal = mu.getNormalMatrix();

	// temporarily override the quad's diffuse with the bg color (incl. alpha)
	float savedDiff[4];
	memcpy(savedDiff, renderer.myMeshes[d.meshID].mat.diffuse, sizeof(savedDiff));
	float bgCol[4] = { bgR, bgG, bgB, bgA };
	memcpy(renderer.myMeshes[d.meshID].mat.diffuse, bgCol, sizeof(bgCol));

	renderer.renderMesh(d);

	// restore material + matrices
	memcpy(renderer.myMeshes[d.meshID].mat.diffuse, savedDiff, sizeof(savedDiff));
	mu.popMatrix(gmu::MODEL);
	mu.popMatrix(gmu::PROJECTION);

	// --- draw the text on top (uses your text shader path) ---
	drawText2D(msg, x, y, scale, textR, textG, textB, textA);

	// --- restore state ---
	if (!blendWasOn) glDisable(GL_BLEND);
	if (depthWasOn)  glEnable(GL_DEPTH_TEST);
}

static void estimateTextSize(const std::string& s, float size, float& outW, float& outH) {
	// tuned to your 2D defaults: ~20 px advance and ~40 px line-height at size=1.0
	outW = (float)s.size() * 0.20f * size; // world units after we scale the quad (see below)
	outH = 0.40f * size;
}

static inline void computeShadowMatrixOnFloor(float S[16]) {
	float plane[4] = { 0.0f, 1.0f, 0.0f, 0.0f };          // y = 0
	float light[4];
	// Use the active sun if available, else fall back to the first point light
	if (directionalLightOn) {
		// Use the *direction* of the light, not its position.
		float lightDir[4] = {
			-directionalLightPos[0],  // negate — shadow projects opposite light direction
			-directionalLightPos[1],
			-directionalLightPos[2],
			 0.0f                     // w = 0 ? directional light (at infinity)
		};
		mu.shadow_matrix(S, plane, lightDir);
	}
	else {
		// For point/spot lights, use the actual position
		float lightPos[4] = {
			pointLightPos[0][0],
			pointLightPos[0][1],
			pointLightPos[0][2],
			pointLightPos[0][3]       // w = 1 ? positional light
		};
		mu.shadow_matrix(S, plane, lightPos);
	}
}

struct ShadowMatGuard {
	MyMesh* m = nullptr;
	float   keepDiff[4]{}, keepAmb[4]{}, keepSpec[4]{}, keepEmi[4]{};
	explicit ShadowMatGuard(MyMesh* mesh, float alpha = 0.9f) : m(mesh) {
		memcpy(keepDiff, m->mat.diffuse, 4 * sizeof(float));
		memcpy(keepAmb, m->mat.ambient, 4 * sizeof(float));
		memcpy(keepSpec, m->mat.specular, 4 * sizeof(float));
		memcpy(keepEmi, m->mat.emissive, 4 * sizeof(float));

		float black[4] = { 0.0f, 0.0f, 0.0f, alpha };
		memcpy(m->mat.diffuse, black, 4 * sizeof(float));
		memcpy(m->mat.ambient, black, 4 * sizeof(float));

		float zero4[4] = { 0,0,0,0 };
		memcpy(m->mat.specular, zero4, 4 * sizeof(float));
		memcpy(m->mat.emissive, zero4, 4 * sizeof(float));
	}
	~ShadowMatGuard() {
		if (!m) return;
		memcpy(m->mat.diffuse, keepDiff, 4 * sizeof(float));
		memcpy(m->mat.ambient, keepAmb, 4 * sizeof(float));
		memcpy(m->mat.specular, keepSpec, 4 * sizeof(float));
		memcpy(m->mat.emissive, keepEmi, 4 * sizeof(float));
	}
};

static void makeBillboardAt(const float pos[3], const Camera& cam, float M[16]) {
	// Compute direction from billboard to camera
	float look[3] = {
		cam.pos[0] - pos[0],
		0.0f, // ignore vertical difference (cylindrical billboard)
		cam.pos[2] - pos[2]
	};
	float len = sqrtf(look[0] * look[0] + look[2] * look[2]);
	if (len < 1e-6f) {
		// fallback
		look[0] = 0.0f; look[2] = 1.0f; len = 1.0f;
	}
	look[0] /= len;
	look[2] /= len;

	// Compute right vector (world up = Y)
	float right[3] = { look[2], 0.0f, -look[0] };

	// Fill the matrix (column-major)
	M[0] = right[0];  M[4] = 0.0f;  M[8] = look[0];  M[12] = pos[0];
	M[1] = right[1];  M[5] = 1.0f;  M[9] = look[1];  M[13] = pos[1];
	M[2] = right[2];  M[6] = 0.0f;  M[10] = look[2];  M[14] = pos[2];
	M[3] = 0.0f;      M[7] = 0.0f;  M[11] = 0.0f;     M[15] = 1.0f;
}

// add params: quadScale, minQuadW, minQuadH
static void renderBillboardLabelAt(const float worldPos[3], const Camera& cam,
	const std::string& msg,
	const float* txtRGBA = nullptr,
	float textSizeWorld = 0.02)
{
	if (!fontLoaded) return;

	// Text color (defaults to white)
	float txCol[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	if (txtRGBA) memcpy(txCol, txtRGBA, 4 * sizeof(float));

	// Billboard matrix that faces the camera (cylindrical)
	float M[16];
	makeBillboardAt(worldPos, cam, M);

	// Estimate text size so we can center it nicely
	float textW, textH;
	estimateTextSize(msg, 1.0f, textW, textH);
	const float worldW = textW * textSizeWorld;
	const float worldH = textH * textSizeWorld;

	mu.pushMatrix(gmu::MODEL);
	mu.loadMatrix(gmu::MODEL, M);

	// Small hover above the beam
	mu.translate(gmu::MODEL, 0.0f, 0.15f * textSizeWorld, 0.0f);

	// Center the text around the billboard origin
	mu.pushMatrix(gmu::MODEL);
	mu.translate(gmu::MODEL, -0.5f * worldW, -0.5f * worldH, 1e-3f); // tiny z to avoid z-fighting
	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);

	// Alpha-blended text
	GLboolean blendWasOn = glIsEnabled(GL_BLEND);
	if (!blendWasOn) glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	TextCommand t{};
	t.str = msg;
	t.position[0] = -0.5f * worldW / textSizeWorld / 0.25;
	t.position[1] = 0.0f;
	t.size = textSizeWorld;
	t.color[0] = txCol[0]; t.color[1] = txCol[1];
	t.color[2] = txCol[2]; t.color[3] = txCol[3];
	t.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	renderer.renderText(t);

	// restore state
	if (!blendWasOn) glDisable(GL_BLEND);

	mu.popMatrix(gmu::MODEL);
	mu.popMatrix(gmu::MODEL);
}




float lastTime = 0.0f;

void renderSmokeParticles(const Camera& cam) {
	if (g_smoke.empty() || smokeQuadMeshID < 0) return;

	// --- make sure correct shader and state are active ---
	renderer.activateRenderMeshesShaderProg();

	GLboolean depthWasOn = glIsEnabled(GL_DEPTH_TEST);
	

	if (!depthWasOn) glEnable(GL_DEPTH_TEST);
	

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // <- correct
	glDepthMask(GL_FALSE);

	for (const auto& p : g_smoke) {
		if (p.life <= 0.0f) continue;

		float ageRatio = 1.0f - (p.life / p.maxLife);
		float fade = 1.0f - ageRatio;
		float scale = p.size * (0.8f + 0.5f * ageRatio);

		mu.pushMatrix(gmu::MODEL);
		mu.translate(gmu::MODEL, p.pos[0], p.pos[1], p.pos[2]);

		float cx = cam.pos[0], cy = cam.pos[1], cz = cam.pos[2];
		float dx = cx - p.pos[0];
		float dz = cz - p.pos[2];
		float len = sqrtf(dx * dx + dz * dz);
		float yaw = (len > 0.0001f) ? atan2f(dx, dz) * 180.0f / 3.14159f : 0.0f;
		mu.rotate(gmu::MODEL, yaw, 0, 1, 0);

		mu.scale(gmu::MODEL, scale, scale, scale);

		mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
		mu.computeNormalMatrix3x3();

		dataMesh d{};
		d.meshID = smokeQuadMeshID;
		d.texMode = 12;
		d.vm = mu.get(gmu::VIEW_MODEL);
		d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
		d.normal = mu.getNormalMatrix();

		renderer.myMeshes[d.meshID].mat.diffuse[3] = fade;

		renderer.renderMesh(d);
		mu.popMatrix(gmu::MODEL);
	}

	glDepthMask(GL_TRUE);
	glDisable(GL_BLEND);

	
	if (!depthWasOn) glDisable(GL_DEPTH_TEST);
}

static void drawSkybox(const Camera& cam)
{
	if (skyboxCubeMeshID < 0) return;      // must have a cube mesh
	renderer.activateRenderMeshesShaderProg();

	// depth: draw behind everything, don’t write depth
	GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);
	if (cullWasOn) glCullFace(GL_FRONT);   // draw inside of cube (or disable cull)
	glDepthMask(GL_FALSE);

	// model at camera position so view*model cancels translation (only rotation remains)
	mu.pushMatrix(gmu::MODEL);
	mu.loadIdentity(gmu::MODEL);
	mu.translate(gmu::MODEL, cam.pos[0], cam.pos[1], cam.pos[2]);
	

	// big cube — just needs to surround the scene
	const float SKYBOX_SIZE = 500.0f;
	mu.scale(gmu::MODEL, SKYBOX_SIZE, SKYBOX_SIZE, SKYBOX_SIZE);
	mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	dataMesh d{};
	d.meshID = skyboxCubeMeshID;
	d.texMode = 10;               // fragment shader should sample the cubemap for this mode
	d.vm = mu.get(gmu::VIEW_MODEL);
	d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	d.normal = mu.getNormalMatrix();

	renderer.renderMesh(d);

	mu.popMatrix(gmu::MODEL);

	glDepthMask(GL_TRUE);
	if (cullWasOn) glCullFace(GL_BACK);
}

static void drawWorldNoHUD_FromCamera(const Camera& cam, float aspect) {
	// ===== Program & global render state =====
	renderer.activateRenderMeshesShaderProg();

	// Fog (same as before)
	if (fogEnabled) {
		int depthFog = 1; float fogColor[3] = { 0.5f, 0.5f, 0.6f }; float fogDensity = 0.04f;
		renderer.setFogParams(depthFog, fogColor, fogDensity);
	}
	else {
		int depthFog = 1; float fogColor[3] = { 0.0f, 0.0f, 0.0f }; float fogDensity = 0.0f;
		renderer.setFogParams(depthFog, fogColor, fogDensity);
	}

	// Texture units (unchanged)
	renderer.setTexUnit(0, 0); renderer.setTexUnit(1, 1); renderer.setTexUnit(2, 2);
	renderer.setTexUnit(3, 3); renderer.setTexUnit(4, 4); renderer.setTexUnit(5, 5);
	renderer.setTexUnit(6, 6); renderer.setTexUnit(7, 7); renderer.setTexUnit(8, 8); renderer.setTexUnit(9, 9);
	renderer.setSkybox(gSkyboxTex, 10);

	// ===== Camera setup (unchanged) =====
	mu.loadIdentity(gmu::VIEW);
	mu.loadIdentity(gmu::MODEL);
	mu.loadIdentity(gmu::PROJECTION);

	if (cam.type == 0) mu.perspective(53.13f, aspect, 0.1f, 1000.0f);
	else {
		float size = 30.0f;
		mu.ortho(-size * aspect, size * aspect, -size, size, 0.1f, 1000.0f);
	}

	mu.loadIdentity(gmu::VIEW);
	mu.lookAt(cam.pos[0], cam.pos[1], cam.pos[2],
		cam.target[0], cam.target[1], cam.target[2],
		0, 1, 0);

	//drawSkybox(cam);

	// =====================================================================
	//                            REFLECTION PASS
	// =====================================================================

	// --- 1) Stencil mark the floor polygon (write 1 where the floor is)
	glEnable(GL_STENCIL_TEST);
	glStencilMask(0xFF);
	glStencilFunc(GL_ALWAYS, 1, 0xFF);
	glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

	GLboolean depthWasOn = glIsEnabled(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_TEST);
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

	drawFloor(); // Just writes into stencil now

	// restore color/depth writes
	if (depthWasOn) glEnable(GL_DEPTH_TEST);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

	// --- 2) Draw mirrored scene where stencil==1
	glStencilMask(0x00);                 // don't modify stencil while drawing
	glStencilFunc(GL_EQUAL, 1, 0xFF);    // only inside the floor silhouette

	GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);
	if (cullWasOn) glDisable(GL_CULL_FACE);   // negative scale flips winding

	setLightsForPass(/*reflected=*/true);      // reflect lights too

	mu.pushMatrix(gmu::MODEL);
	mu.scale(gmu::MODEL, 1.0f, -1.0f, 1.0f);  // mirror across y=0

	// ----------------------
	// draw SCENE (no floor)
	// ----------------------
	{
		dataMesh data{};
		const float spacing = 7.2f;
		const float floorSize = 65.0f;
		const float xOffset = -floorSize / 2.0f + spacing / 2.0f;
		const float zOffset = -floorSize / 2.0f + spacing / 2.0f;

		// OPAQUE BUILDINGS
		for (int i = 0; i < GRID_SIZE; ++i) {
			for (int j = 0; j < GRID_SIZE; ++j) {
				int mID = city[i][j].meshID;
				if (mID == glassCubeMeshID || mID == glassCylMeshID) continue;

				mu.pushMatrix(gmu::MODEL);
				mu.translate(gmu::MODEL,
					xOffset + (float)i * spacing + buildingOffset[i][j][0],
					0.0f + buildingOffset[i][j][1],
					zOffset + (float)j * spacing + buildingOffset[i][j][2]);

				float height = city[i][j].height;
				if (mID == 3) {
					mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
					mu.translate(gmu::MODEL, 0.0f, 0.75f, 0.0f);
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

		// TRANSPARENT (GLASS) BUILDINGS
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		for (int i = 0; i < GRID_SIZE; ++i) {
			for (int j = 0; j < GRID_SIZE; ++j) {
				int mID = city[i][j].meshID;
				if (mID != glassCubeMeshID && mID != glassCylMeshID) continue;

				mu.pushMatrix(gmu::MODEL);
				mu.translate(gmu::MODEL,
					xOffset + (float)i * spacing + buildingOffset[i][j][0],
					0.0f + buildingOffset[i][j][1],
					zOffset + (float)j * spacing + buildingOffset[i][j][2]);

				float height = city[i][j].height;
				if (mID == glassCylMeshID) {
					mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
					mu.translate(gmu::MODEL, 0.0f, 0.75f, 0.0f);
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
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);

		// PACKAGE
		if (package.active && packageMeshID >= 0) {
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, package.pos[0], package.pos[1], package.pos[2]);
			mu.translate(gmu::MODEL, -0.5f, 0.0f, -0.5f);
			mu.scale(gmu::MODEL, 0.7f, 0.7f, 0.7f);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();

			data.meshID = packageMeshID;
			data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);

			mu.popMatrix(gmu::MODEL);
		}

		// DRONE (body + rotors)
		{
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
			mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f);
			mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f);
			mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f);

			dataMesh data{};

			// body
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f);
			mu.scale(gmu::MODEL, 2.0f, 0.6f, 2.0f);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = droneBodyMeshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);

			// rotors
			const float halfX = 2.0f * 0.5f;
			const float halfZ = 2.0f * 0.5f;
			const float rotorH = 0.2f;
			for (int ix = -1; ix <= 1; ix += 2) {
				for (int iz = -1; iz <= 1; iz += 2) {
					mu.pushMatrix(gmu::MODEL);
					mu.translate(gmu::MODEL, ix * halfX, rotorH, iz * halfZ);
					mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
					mu.scale(gmu::MODEL, 1.7f, 0.3f, 1.7f);
					mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
					mu.computeNormalMatrix3x3();
					data.meshID = droneRotorMeshID; data.texMode = 0;
					data.vm = mu.get(gmu::VIEW_MODEL);
					data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
					data.normal = mu.getNormalMatrix();
					renderer.renderMesh(data);
					mu.popMatrix(gmu::MODEL);
				}
			}

			mu.popMatrix(gmu::MODEL);
		}

		// FLYING OBJECTS
		for (const auto& o : flyingObjects) {
			dataMesh data{};
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
			if (o.rotAxis == 0)      mu.rotate(gmu::MODEL, o.rotAngle, 1, 0, 0);
			else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0, 1, 0);
			else                     mu.rotate(gmu::MODEL, o.rotAngle, 0, 0, 1);
			mu.scale(gmu::MODEL, o.size, o.size, o.size);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = o.meshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);
		}

		// BEAM (no label in reflection)
		if (beamMeshID >= 0) {
			const float* beamPos = (!package.beingCarried) ? package.pos : package.destPos;
			float beamHeight = 25.0f, beamRadius = 3.0f;

			renderer.activateRenderMeshesShaderProg();
			glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE);
			glDepthMask(GL_FALSE); glEnable(GL_DEPTH_TEST);

			dataMesh data{};
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, beamPos[0], beamPos[1] + beamHeight * 0.5f, beamPos[2]);
			mu.scale(gmu::MODEL, beamRadius, beamHeight, beamRadius);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = beamMeshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);

			glDepthMask(GL_TRUE);
			glDisable(GL_BLEND);
		}
	}
	// ----------------------
	// end reflected scene
	// ----------------------
	mu.popMatrix(gmu::MODEL);      // pop (1,-1,1)
	if (cullWasOn) glEnable(GL_CULL_FACE);

	// --- 3) Draw the floor semi-transparent over the reflection
	{
		{
			float saved[4]; memcpy(saved, renderer.myMeshes[0].mat.diffuse, sizeof(saved));
			renderer.myMeshes[0].mat.diffuse[3] = 0.6f; // 40% see-through

			// Enable stencil constraint again — only draw where reflection exists
			glEnable(GL_STENCIL_TEST);
			glStencilFunc(GL_EQUAL, 1, 0xFF); // only where reflection was drawn
			glStencilMask(0x00);              // don’t modify stencil buffer

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			// Disable depth writes so the floor doesn’t occlude other objects later
			

			glDepthFunc(GL_LEQUAL);
			drawFloor(6);
			glDepthFunc(GL_LESS);

			glDisable(GL_BLEND);
			glDisable(GL_STENCIL_TEST);

			memcpy(renderer.myMeshes[0].mat.diffuse, saved, sizeof(saved));
		}
	}

	// We’re done using stencil for reflection
	glDisable(GL_STENCIL_TEST);
	glStencilMask(0xFF);

	// =====================================================================
	//                       SHADOWS OVER THE FLOOR
	//  (drawn AFTER the floor so they darken the reflection underneath)
	// =====================================================================
	{
		// reuse your existing shadow block exactly, just placed here:
		float S[16];
		computeShadowMatrixOnFloor(S);

		GLboolean cWasOn = glIsEnabled(GL_CULL_FACE);
		if (cWasOn) glDisable(GL_CULL_FACE);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(-1.0f, -1.0f);
		glDepthMask(GL_FALSE);

		dataMesh sd{}; sd.texMode = 9; // flat material color path in shader

		// Drone body shadow
		if (droneBodyMeshID >= 0) {
			ShadowMatGuard matGuard(&renderer.myMeshes[droneBodyMeshID], 0.80f);
			mu.pushMatrix(gmu::MODEL);
			mu.multMatrix(gmu::MODEL, S);
			mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
			mu.rotate(gmu::MODEL, drone.dirAngle, 0, 1, 0);
			mu.rotate(gmu::MODEL, drone.pitch, 1, 0, 0);
			mu.rotate(gmu::MODEL, drone.roll, 0, 0, 1);
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f);
			mu.scale(gmu::MODEL, 2.0f, 0.6f, 2.0f);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			sd.meshID = droneBodyMeshID;
			sd.vm = mu.get(gmu::VIEW_MODEL);
			sd.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			sd.normal = mu.getNormalMatrix();
			renderer.renderMesh(sd);
			mu.popMatrix(gmu::MODEL);
			mu.popMatrix(gmu::MODEL);
		}

		// Drone rotor shadows
		if (droneRotorMeshID >= 0) {
			ShadowMatGuard matGuard(&renderer.myMeshes[droneRotorMeshID], 0.45f);
			const float rotorHeight = 0.2f;
			const float bodyScaleX = 2.0f, bodyScaleZ = 2.0f;
			const float halfX = bodyScaleX * 0.5f;
			const float halfZ = bodyScaleZ * 0.5f;

			mu.pushMatrix(gmu::MODEL);
			mu.multMatrix(gmu::MODEL, S);
			mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
			mu.rotate(gmu::MODEL, drone.dirAngle, 0, 1, 0);
			mu.rotate(gmu::MODEL, drone.pitch, 1, 0, 0);
			mu.rotate(gmu::MODEL, drone.roll, 0, 0, 1);

			for (int ix = -1; ix <= 1; ix += 2) {
				for (int iz = -1; iz <= 1; iz += 2) {
					mu.pushMatrix(gmu::MODEL);
					mu.translate(gmu::MODEL, ix * halfX, rotorHeight, iz * halfZ);
					mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
					mu.scale(gmu::MODEL, 1.7f, 0.3f, 1.7f);
					mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
					mu.computeNormalMatrix3x3();
					sd.meshID = droneRotorMeshID;
					sd.vm = mu.get(gmu::VIEW_MODEL);
					sd.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
					sd.normal = mu.getNormalMatrix();
					renderer.renderMesh(sd);
					mu.popMatrix(gmu::MODEL);
				}
			}
			mu.popMatrix(gmu::MODEL);
		}

		// Flying object shadows
		for (const auto& o : flyingObjects) {
			if (o.meshID < 0) continue;
			ShadowMatGuard matGuard(&renderer.myMeshes[o.meshID], 0.45f);
			mu.pushMatrix(gmu::MODEL);
			mu.multMatrix(gmu::MODEL, S);
			mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
			if (o.rotAxis == 0)      mu.rotate(gmu::MODEL, o.rotAngle, 1, 0, 0);
			else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0, 1, 0);
			else                     mu.rotate(gmu::MODEL, o.rotAngle, 0, 0, 1);
			mu.scale(gmu::MODEL, o.size, o.size, o.size);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			sd.meshID = o.meshID;
			sd.vm = mu.get(gmu::VIEW_MODEL);
			sd.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			sd.normal = mu.getNormalMatrix();
			renderer.renderMesh(sd);
			mu.popMatrix(gmu::MODEL);
		}

		glDepthMask(GL_TRUE);
		glDisable(GL_POLYGON_OFFSET_FILL);
		glDisable(GL_BLEND);
		if (cWasOn) glEnable(GL_CULL_FACE);
	}

	// =====================================================================
	//                         NORMAL (UNMIRRORED) SCENE
	// =====================================================================

	setLightsForPass(/*reflected=*/false);

	{
		dataMesh data{};
		const float spacing = 7.2f;
		const float floorSize = 65.0f;
		const float xOffset = -floorSize / 2.0f + spacing / 2.0f;
		const float zOffset = -floorSize / 2.0f + spacing / 2.0f;

		// OPAQUE BUILDINGS
		for (int i = 0; i < GRID_SIZE; ++i) {
			for (int j = 0; j < GRID_SIZE; ++j) {
				int mID = city[i][j].meshID;
				if (mID == glassCubeMeshID || mID == glassCylMeshID) continue;

				mu.pushMatrix(gmu::MODEL);
				mu.translate(gmu::MODEL,
					xOffset + (float)i * spacing + buildingOffset[i][j][0],
					0.0f + buildingOffset[i][j][1],
					zOffset + (float)j * spacing + buildingOffset[i][j][2]);

				float height = city[i][j].height;
				if (mID == 3) {
					mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
					mu.translate(gmu::MODEL, 0.0f, 0.75f, 0.0f);
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

		// TRANSPARENT (GLASS) BUILDINGS
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);
		for (int i = 0; i < GRID_SIZE; ++i) {
			for (int j = 0; j < GRID_SIZE; ++j) {
				int mID = city[i][j].meshID;
				if (mID != glassCubeMeshID && mID != glassCylMeshID) continue;

				mu.pushMatrix(gmu::MODEL);
				mu.translate(gmu::MODEL,
					xOffset + (float)i * spacing + buildingOffset[i][j][0],
					0.0f + buildingOffset[i][j][1],
					zOffset + (float)j * spacing + buildingOffset[i][j][2]);

				float height = city[i][j].height;
				if (mID == glassCylMeshID) {
					mu.scale(gmu::MODEL, 2.3f, height, 2.3f);
					mu.translate(gmu::MODEL, 0.0f, 0.75f, 0.0f);
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
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);

		// PACKAGE
		if (package.active && packageMeshID >= 0) {
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, package.pos[0], package.pos[1], package.pos[2]);
			mu.translate(gmu::MODEL, -0.5f, 0.0f, -0.5f);
			mu.scale(gmu::MODEL, 0.7f, 0.7f, 0.7f);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();

			data.meshID = packageMeshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);

			mu.popMatrix(gmu::MODEL);
		}

		// DRONE
		{
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, drone.pos[0], drone.pos[1], drone.pos[2]);
			mu.rotate(gmu::MODEL, drone.dirAngle, 0.0f, 1.0f, 0.0f);
			mu.rotate(gmu::MODEL, drone.pitch, 1.0f, 0.0f, 0.0f);
			mu.rotate(gmu::MODEL, drone.roll, 0.0f, 0.0f, 1.0f);

			dataMesh data{};

			// body
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, -0.5f, -0.5f, -0.5f);
			mu.scale(gmu::MODEL, 2.0f, 0.6f, 2.0f);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = droneBodyMeshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);

			// rotors
			const float halfX = 2.0f * 0.5f;
			const float halfZ = 2.0f * 0.5f;
			const float rotorH = 0.2f;
			for (int ix = -1; ix <= 1; ix += 2) {
				for (int iz = -1; iz <= 1; iz += 2) {
					mu.pushMatrix(gmu::MODEL);
					mu.translate(gmu::MODEL, ix * halfX, rotorH, iz * halfZ);
					mu.translate(gmu::MODEL, 0.5f, 0.0f, 0.5f);
					mu.scale(gmu::MODEL, 1.7f, 0.3f, 1.7f);
					mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
					mu.computeNormalMatrix3x3();
					data.meshID = droneRotorMeshID; data.texMode = 0;
					data.vm = mu.get(gmu::VIEW_MODEL);
					data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
					data.normal = mu.getNormalMatrix();
					renderer.renderMesh(data);
					mu.popMatrix(gmu::MODEL);
				}
			}

			mu.popMatrix(gmu::MODEL);
		}

		// FLYING OBJECTS
		for (const auto& o : flyingObjects) {
			dataMesh data{};
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, o.pos[0], o.pos[1], o.pos[2]);
			if (o.rotAxis == 0)      mu.rotate(gmu::MODEL, o.rotAngle, 1, 0, 0);
			else if (o.rotAxis == 1) mu.rotate(gmu::MODEL, o.rotAngle, 0, 1, 0);
			else                     mu.rotate(gmu::MODEL, o.rotAngle, 0, 0, 1);
			mu.scale(gmu::MODEL, o.size, o.size, o.size);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = o.meshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);
		}

		// BEAM + BILLBOARD LABEL (normal, *not* reflected)
		if (beamMeshID >= 0) {
			float beamHeight = 25.0f;
			float beamRadius = 3.0f;
			const float* beamPos = (!package.beingCarried) ? package.pos : package.destPos;

			renderer.activateRenderMeshesShaderProg();
			glEnable(GL_BLEND);
			glBlendFunc(GL_ONE, GL_ONE);
			glDepthMask(GL_FALSE);
			glEnable(GL_DEPTH_TEST);

			dataMesh data{};
			mu.pushMatrix(gmu::MODEL);
			mu.translate(gmu::MODEL, beamPos[0], beamPos[1] + beamHeight * 0.5f, beamPos[2]);
			mu.scale(gmu::MODEL, beamRadius, beamHeight, beamRadius);
			mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
			mu.computeNormalMatrix3x3();
			data.meshID = beamMeshID; data.texMode = 0;
			data.vm = mu.get(gmu::VIEW_MODEL);
			data.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
			data.normal = mu.getNormalMatrix();
			renderer.renderMesh(data);
			mu.popMatrix(gmu::MODEL);

			glDepthMask(GL_TRUE);
			glDisable(GL_BLEND);

			// billboard label above beam
			float labelPos[3] = { beamPos[0], beamPos[1] + beamHeight * 1.05f, beamPos[2] };
			std::string label = (!package.beingCarried) ? "Pick up here" : "Deliver here";
			renderBillboardLabelAt(labelPos, cam, label, nullptr, 0.02f);
		}
	}

	// SMOKE (normal pass only)
	renderSmokeParticles(cams[activeCam]);

	glDepthFunc(GL_LEQUAL); // allow skybox behind everything
	drawSkybox(cam);
	glDepthFunc(GL_LESS);
}

static void drawHudMaskBorderCore(float cx, float cy, float sizePx) {
	int vp[4]; glGetIntegerv(GL_VIEWPORT, vp);
	const float W = (float)vp[2], H = (float)vp[3];

	// --- HUD state: no depth, no stencil; blend ok; no cull; no depth writes
	GLboolean depthWasOn = glIsEnabled(GL_DEPTH_TEST);
	GLboolean stencilWasOn = glIsEnabled(GL_STENCIL_TEST);
	GLboolean blendWasOn = glIsEnabled(GL_BLEND);
	GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);

	if (stencilWasOn) glDisable(GL_STENCIL_TEST);
	if (depthWasOn)   glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	if (!blendWasOn)  glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	if (cullWasOn)    glDisable(GL_CULL_FACE);

	renderer.activateRenderMeshesShaderProg();

	mu.pushMatrix(gmu::PROJECTION);
	mu.loadIdentity(gmu::PROJECTION);
	mu.ortho(0.0f, W, 0.0f, H, -1.0f, 1.0f);

	mu.loadIdentity(gmu::VIEW);
	mu.loadIdentity(gmu::MODEL);
	mu.translate(gmu::MODEL, cx, cy, 0.0f);
	mu.scale(gmu::MODEL, sizePx, sizePx, 1.0f);

	mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
	mu.computeNormalMatrix3x3();

	// temporarily color it
	float savedDiffuse[4];
	memcpy(savedDiffuse, renderer.myMeshes[stencilMaskMeshID].mat.diffuse, sizeof(savedDiffuse));
	float yellow[4] = { 1,1,0,1 };
	memcpy(renderer.myMeshes[stencilMaskMeshID].mat.diffuse, yellow, sizeof(yellow));

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glLineWidth(2.0f);

	dataMesh d{};
	d.meshID = stencilMaskMeshID;
	d.texMode = 0;
	d.vm = mu.get(gmu::VIEW_MODEL);
	d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
	d.normal = mu.getNormalMatrix();
	renderer.renderMesh(d);

	// restore
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	memcpy(renderer.myMeshes[stencilMaskMeshID].mat.diffuse, savedDiffuse, sizeof(savedDiffuse));
	mu.popMatrix(gmu::PROJECTION);

	if (cullWasOn)    glEnable(GL_CULL_FACE);
	if (!blendWasOn)  glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	if (depthWasOn)   glEnable(GL_DEPTH_TEST);
	if (stencilWasOn) glEnable(GL_STENCIL_TEST);
}


static bool worldToScreen_activeCam(const float world[4], int& outX, int& outY) {
	int vp[4]; glGetIntegerv(GL_VIEWPORT, vp);

	mu.pushMatrix(gmu::PROJECTION);
	mu.pushMatrix(gmu::VIEW);

	mu.loadIdentity(gmu::PROJECTION);
	mu.perspective(53.13f, aspectRatio, 0.1f, 1000.0f);

	mu.loadIdentity(gmu::VIEW);
	mu.lookAt(cams[activeCam].pos[0], cams[activeCam].pos[1], cams[activeCam].pos[2],
		cams[activeCam].target[0], cams[activeCam].target[1], cams[activeCam].target[2], 0, 1, 0);

	// ---- FIX: copy to a mutable vector (ensure w = 1 for points, 0 for directions)
	float wv[4] = { world[0], world[1], world[2], world[3] };

	float eye[4];  mu.multMatrixPoint(gmu::VIEW, wv, eye);
	float clip[4]; mu.multMatrixPoint(gmu::PROJECTION, eye, clip);

	mu.popMatrix(gmu::VIEW);
	mu.popMatrix(gmu::PROJECTION);

	if (fabsf(clip[3]) < 1e-6f) return false;

	float ndcX = clip[0] / clip[3];
	float ndcY = clip[1] / clip[3];

	outX = (int)(vp[0] + (ndcX * 0.5f + 0.5f) * vp[2]);
	outY = (int)(vp[1] + (ndcY * 0.5f + 0.5f) * vp[3]);
	return true;
}


void render_flare(FLARE_DEF* flare, int lx, int ly, int* m_viewport) {
	auto clampi = [](int x, int a, int b) { return x < a ? a : (x > b ? b : x); };
	
	// Save GL state and set HUD blending
	GLboolean depthWasOn = glIsEnabled(GL_DEPTH_TEST);
	GLboolean cullWasOn = glIsEnabled(GL_CULL_FACE);
	GLboolean blendWasOn = glIsEnabled(GL_BLEND);
	if (depthWasOn) glDisable(GL_DEPTH_TEST);
	if (cullWasOn)  glDisable(GL_CULL_FACE);
	if (!blendWasOn) glEnable(GL_BLEND);

	glBlendEquation(GL_FUNC_ADD);
	glBlendFunc(GL_ONE, GL_ONE);
	glDepthMask(GL_FALSE);

	const int x0 = m_viewport[0], y0 = m_viewport[1];
	const int W = m_viewport[2], H = m_viewport[3];
	const int xmax = x0 + W - 1, ymax = y0 + H - 1;

	const int cx = x0 + (int)(0.5f * W) - 1;
	const int cy = y0 + (int)(0.5f * H) - 1;

	const float maxflaredist = sqrtf((float)(cx * cx + cy * cy));
	const float flaredist = sqrtf((float)((lx - cx) * (lx - cx) + (ly - cy) * (ly - cy)));
	const float scaleDistance = (maxflaredist - flaredist) / maxflaredist;
	const float flareMaxSize = (float)W * flare->fMaxSize;
	const float flareScale = (float)W * flare->fScale;

	// destination (mirror point across screen center)
	const int dx = clampi(cx + (cx - lx), x0, xmax);
	const int dy = clampi(cy + (cy - ly), y0, ymax);

	// HUD ortho
	renderer.activateRenderMeshesShaderProg();
	mu.pushMatrix(gmu::PROJECTION);
	mu.loadIdentity(gmu::PROJECTION);
	mu.ortho(0.0f, (float)W, 0.0f, (float)H, -1.0f, 1.0f);

	mu.loadIdentity(gmu::VIEW);

	// Ensure shader samplers think TU9 == 9 (you already do this earlier each frame)
	// renderer.setTexUnit(9, 9);

	for (int i = 0; i < flare->nPieces; ++i) {
		// position along center<->dest line
		int px = (int)((1.0f - flare->element[i].fDistance) * lx + flare->element[i].fDistance * dx);
		int py = (int)((1.0f - flare->element[i].fDistance) * ly + flare->element[i].fDistance * dy);
		px = clampi(px, x0, xmax);
		py = clampi(py, y0, ymax);

		int width = (int)(scaleDistance * flareScale * flare->element[i].fSize);
		if (width > (int)flareMaxSize) width = (int)flareMaxSize;
		int height = (int)((float)H / (float)W * (float)width);
		if (width <= 1) continue;

		// set color (diffuse) with alpha scaled by distance
		float diffuse[4];
		memcpy(diffuse, flare->element[i].matDiffuse, sizeof(diffuse));
		diffuse[3] *= scaleDistance;

		// Bind the flare texture to TU9 (the shader will sample texmap9 in texMode 7)
		glActiveTexture(GL_TEXTURE9);
		glBindTexture(GL_TEXTURE_2D, FlareTextureArray[flare->element[i].textureId]);

		// Place & scale a 1x1 quad (we reuse the smoke billboard quad)
		mu.pushMatrix(gmu::MODEL);
		mu.translate(gmu::MODEL, (float)px, (float)py, 0.0f);  // bottom-left anchor (matches template)
		mu.scale(gmu::MODEL, (float)width, (float)height, 1.0f);

		mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
		mu.computeNormalMatrix3x3();

		dataMesh d{};
		
		d.meshID = flareQuadMeshID();   // smoke quad
		d.texMode = 7;                  // "sprite * mat.diffuse"
		d.vm = mu.get(gmu::VIEW_MODEL);
		d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
		d.normal = mu.getNormalMatrix();

		// push the color to this mesh for this draw
		memcpy(renderer.myMeshes[d.meshID].mat.diffuse, diffuse, sizeof(diffuse));

		renderer.renderMesh(d);
		mu.popMatrix(gmu::MODEL);
	}

	mu.popMatrix(gmu::PROJECTION);

	// restore GL state
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_TRUE);
	if (!blendWasOn) glDisable(GL_BLEND);
	if (cullWasOn)   glEnable(GL_CULL_FACE);
	if (depthWasOn)  glEnable(GL_DEPTH_TEST);
}

void renderSim(void) {

	FrameCount++;

	float currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0f;
	float deltaTime = currentTime - lastTime;
	lastTime = currentTime;

	if (!paused) {
		animate(deltaTime);
	}

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

	glStencilMask(0xFF);
	glClearStencil(0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	if (showRearCam) {
		// --- STENCIL MASK PASS (screen-space quad writes 1s) ---
		glStencilMask(0xFF);
		glDisable(GL_DEPTH_TEST);
		glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
		glEnable(GL_STENCIL_TEST);
		glStencilFunc(GL_ALWAYS, 1, 0xFF);
		glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

		// compute inset rect (in pixels)
		int vp[4];
		glGetIntegerv(GL_VIEWPORT, vp);
		float W = float(vp[2]), H = float(vp[3]);
		float size = rearMaskSizePx, margin = rearMaskMarginPx;

		// center coordinates of inset mask
		float cx = W - margin - size * 0.5f;
		float cy = margin + size * 0.5f;

		// --- define inset viewport region (square) ---
		int insetW = (int)size;
		int insetH = (int)size;
		int insetX = (int)(W - margin - size);
		int insetY = (int)margin;

		// render the mask quad in HUD-ortho via your renderer
		renderer.activateRenderMeshesShaderProg();
		mu.pushMatrix(gmu::PROJECTION);
		mu.loadIdentity(gmu::PROJECTION);
		mu.ortho(0.0f, W, 0.0f, H, -1.0f, 1.0f);

		mu.loadIdentity(gmu::VIEW);
		mu.loadIdentity(gmu::MODEL);
		mu.translate(gmu::MODEL, cx, cy, 0.0f);
		mu.scale(gmu::MODEL, size, size, 1.0f);

		mu.computeDerivedMatrix(gmu::PROJ_VIEW_MODEL);
		mu.computeNormalMatrix3x3();

		dataMesh d{};
		d.meshID = stencilMaskMeshID; // your 1x1 quad
		d.texMode = 0;
		d.vm = mu.get(gmu::VIEW_MODEL);
		d.pvm = mu.get(gmu::PROJ_VIEW_MODEL);
		d.normal = mu.getNormalMatrix();
		renderer.renderMesh(d);

		mu.popMatrix(gmu::PROJECTION);

		// restore write masks (keep stencil test enabled for next passes)
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glEnable(GL_DEPTH_TEST);
		glStencilMask(0x00); // don't change stencil during scene

		// --- MAIN WORLD PASS (full screen / active camera) ---
		// Stencil has 1s in the inset; we just render normally here.
		glStencilMask(0x00);            // don’t modify stencil
		glStencilFunc(GL_NOTEQUAL, 1, 0xFF);  // draw everywhere EXCEPT where stencil == 1

		drawWorldNoHUD_FromCamera(cams[activeCam], aspectRatio);


		// --- INSET 3D PASS (top-down) INSIDE STENCIL, with its own viewport & depth ---
		Camera top = cams[2];
		top.type = 0; // perspective

		// Look from above, but avoid forward // up colinearity by adding a tiny horizontal component
		top.pos[0] = drone.pos[0];
		top.pos[1] = drone.pos[1] + 40.0f;
		top.pos[2] = drone.pos[2];

		top.target[0] = drone.pos[0];
		top.target[1] = drone.pos[1] - 5.0f;   // slight tilt down instead of exactly at the drone Y
		top.target[2] = drone.pos[2] + 0.01f;  // tiny Z offset to break colinearity with up=(0,1,0) 

		glStencilFunc(GL_EQUAL, 1, 0xFF);

		// set inset viewport (square) so the projection aspect=1.0 looks right
		int vpFull[4]; glGetIntegerv(GL_VIEWPORT, vpFull);
		glViewport((GLint)insetX, (GLint)insetY, insetW, insetH);

		// clear the DEPTH BUFFER ONLY inside the inset region, so inset has correct 3D
		glEnable(GL_SCISSOR_TEST);
		glScissor((GLint)insetX, (GLint)insetY, insetW, insetH);
		glClear(GL_DEPTH_BUFFER_BIT);
		glDisable(GL_SCISSOR_TEST);

		// proper 3D inside inset
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);

		// draw inset with square aspect
		drawWorldNoHUD_FromCamera(top, 1.0f);

		// restore full viewport & state
		glViewport(vpFull[0], vpFull[1], vpFull[2], vpFull[3]);
		glDisable(GL_STENCIL_TEST);
		glStencilMask(0xFF);
	}

	if (flareEffect && pointLightsOn && !spotLightsOn) {
		int vp[4]; glGetIntegerv(GL_VIEWPORT, vp);

		// Choose one point light to “flare”—template uses a single point light.
		float Lw[4] = { pointLightPos[0][0], pointLightPos[0][1], pointLightPos[0][2], 1.0f };
		int lx = 0, ly = 0;
		if (worldToScreen_activeCam(Lw, lx, ly)) {
			render_flare(&gFlare, lx, ly, vp);
		}
	}
	//Render text (bitmap fonts) in screen coordinates. So use ortoghonal projection with viewport coordinates.
	//Each glyph quad texture needs just one byte color channel: 0 in background and 1 for the actual character pixels. Use it for alpha blending
	//text to be rendered in last place to be in front of everything
	
	// -- GAME OVER trigger --
	if (batteryLevel <= 0.0f && !gameOver) {
		gameOver = true;
	}

	glDisable(GL_STENCIL_TEST);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	drawTextWithBackground("2025 Drone Project", 100, 200, 0.5f,
		1.0f, 1.0f, 1.0f, 1.0f,      // text color
		0.0f, 0.0f, 0.0f, 0.5f);

	if (showRearCam) {
		int vpHUD[4]; glGetIntegerv(GL_VIEWPORT, vpHUD);
		float W = (float)vpHUD[2], H = (float)vpHUD[3];
		float size = rearMaskSizePx, margin = rearMaskMarginPx;
		float cx = W - margin - size * 0.5f;
		float cy = margin + size * 0.5f;
		drawHudMaskBorderCore(cx, cy, size);
	}

	if (paused) {
		int textX = WinX / 2 - 100;
		int textY = WinY / 2;
		drawText2D("PAUSED", textX, textY, 1.2f, 1.0f, 0.0f, 0.0f);
	}

	//  HUD (Battery + Score)
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	// marge depuis le haut de la fenêtre
	int marginTop = 20;
	int hudHeight = 18;

	// y mesuré depuis le haut de l’écran
	int hudY_from_top = marginTop;

	drawText2D("Battery", 20, viewport[3] - hudY_from_top, 0.6f, 1, 1, 1);
	renderer.drawBatteryHUD(batteryLevel, 100, hudY_from_top, 200, hudHeight);
	drawText2D(std::to_string((int)batteryLevel) + "%", 320, viewport[3] - hudY_from_top + 5, 0.6f, 1, 1, 1);


	//drawText2D("Score: " + std::to_string(score), 20, hudY - 40, 0.6f, 1.0f, 1.0f, 0.0f);

	//  GAME OVER screen
	if (gameOver) {
		int textX = WinX / 2 - 180;
		int textY = WinY / 2;
		drawText2D("GAME OVER", textX, textY, 1.5f, 1.0f, 0.0f, 0.0f);
		drawText2D("Press R to restart", textX - 40, textY - 60, 0.8f, 1.0f, 1.0f, 1.0f);
	}

	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

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

	case 'c':
		pointLightsOn = !pointLightsOn;
		if (pointLightsOn) directionalLightOn = false;
		break;
	case 'h': spotLightsOn = !spotLightsOn; break;

		/*case 'l':
			spotlight_mode = !spotlight_mode;
			printf(spotlight_mode ? "Point light disabled. Spot light enabled\n"
				: "Spot light disabled. Point light enabled\n");
			break;*/

			/*case 'r':
				alpha = 57.0f; beta = 18.0f; r = 45.0f;
				camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
				camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
				camY = r * sin(beta * 3.14f / 180.0f);
				break;*/

	case 'm': glEnable(GL_MULTISAMPLE); break;
	case 'n': 
		directionalLightOn = !directionalLightOn;
		if (directionalLightOn) pointLightsOn = false; // ensure only one main source
		break;
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

	case 'p':
		paused = !paused; // toggle pause
		break;

	case 'r':
		if (gameOver) {
			resetDrone();
			batteryLevel = 100.0f;
			gameOver = false;
		}
		break;

	case 'l':
		if (!spotLightsOn) 
			flareEffect = !flareEffect;
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
	if (state == GLUT_DOWN) {
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
	camY = r * sin(beta * 3.14f / 180.0f);

	//  uncomment this if not using an idle or refresh func
	//	glutPostRedisplay();
}


static GLuint loadTextureDevIL(const char* path) {
	ILuint img = 0;
	ilGenImages(1, &img);
	ilBindImage(img);

	if (!ilLoadImage((ILstring)path)) {
		ilDeleteImages(1, &img);
		fprintf(stderr, "DevIL failed to load %s\n", path);
		return 0;
	}

	ilConvertImage(IL_RGBA, IL_UNSIGNED_BYTE);
	int w = ilGetInteger(IL_IMAGE_WIDTH);
	int h = ilGetInteger(IL_IMAGE_HEIGHT);
	const void* data = ilGetData();

	GLuint tex = 0;
	glGenTextures(1, &tex);
	glBindTexture(GL_TEXTURE_2D, tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	ilDeleteImages(1, &img);
	return tex;
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
	renderer.TexObjArray.texture2D_Loader("assets/smoke_particle.png");

	const char* skyFaces[6] = {
	"assets/stormydays_rt.tga", // +X (right)
	"assets/stormydays_lf.tga", // -X (left)
	"assets/stormydays_up.tga", // +Y (top)
	"assets/stormydays_dn.tga", // -Y (bottom)
	"assets/stormydays_ft.tga", // +Z (front)
	"assets/stormydays_bk.tga"  // -Z (back)
	};
	renderer.TexObjArray.textureCubeMap_Loader(skyFaces);
	GLuint cubeIndex = renderer.TexObjArray.getNumTextureObjects() - 1;
	gSkyboxTex = renderer.TexObjArray.getTextureId(cubeIndex);

	setSunFromAngles(sunAzimuthDeg, sunElevationDeg);
	//Scene geometry with triangle meshes

	MyMesh amesh;

	float amb[] = { 0.2f, 0.15f, 0.1f, 1.0f };
	float diff[] = { 0.8f, 0.6f, 0.4f, 1.0f };
	float spec[] = { 0.8f, 0.8f, 0.8f, 1.0f };

	float amb1[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	float diff1[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	float spec1[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	float emissive[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float shininess = 100.0f;
	int texcount = 0;

	float ambNeutral[] = { 0.20f, 0.20f, 0.20f, 1.0f };

	// create geometry and VAO of the floor quad
	amesh = createQuad(65.0f, 65.0f);
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



	// Drone body(flattened cube)
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

	// Drone rotor (cylinder)
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
		MyMesh mask = createQuad(1.0f, 1.0f);
		mask.mat.texCount = 0;
		renderer.myMeshes.push_back(mask);
		stencilMaskMeshID = (int)renderer.myMeshes.size() - 1;
	}

	// Wireframe cube for AABB debug drawing
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

	{
		MyMesh sky = createCube();
		float amb[] = { 0,0,0,1 }, diff[] = { 1,1,1,1 }, spec[] = { 0,0,0,1 }, emis[] = { 0,0,0,1 };
		sky.mat.shininess = 0.0f; sky.mat.texCount = 0; // material irrelevant for cubemap
		memcpy(sky.mat.ambient, amb, 4 * sizeof(float));
		memcpy(sky.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(sky.mat.specular, spec, 4 * sizeof(float));
		memcpy(sky.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(sky);
		skyboxCubeMeshID = (int)renderer.myMeshes.size() - 1;
	}

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

	spawnPackage();

	// set the camera position based on its spherical coordinates
	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);

	// top-down perspective
	cams[0].pos[0] = 0.0f;
	cams[0].pos[1] = drone.pos[1] + 50.0f;     // high above
	cams[0].pos[2] = drone.pos[2] + 0.01f;     // slight offset to avoid singularity
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

	{
		MyMesh pkg = createCube();
		float amb[] = { 0.3f, 0.3f, 0.0f, 1.0f }; // yellowish
		float diff[] = { 0.9f, 0.8f, 0.1f, 1.0f };
		float spec[] = { 0.8f, 0.8f, 0.3f, 1.0f };
		float emis[] = { 0.1f, 0.1f, 0.0f, 1.0f };
		pkg.mat.shininess = 60.0f; pkg.mat.texCount = 0;
		memcpy(pkg.mat.ambient, amb, 4 * sizeof(float));
		memcpy(pkg.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(pkg.mat.specular, spec, 4 * sizeof(float));
		memcpy(pkg.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(pkg);
		packageMeshID = (int)renderer.myMeshes.size() - 1;
	}

	{
		MyMesh beam = createCylinder(1.0f, 0.3f, 16);
		float amb[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		float diff[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		float spec[] = { 0.0f, 0.0f, 0.0f, 1.0f };
		float emis[] = { 2.0f, 1.6f, 0.4f, 1.0f }; // bright yellowish
		beam.mat.shininess = 1.0f;
		beam.mat.texCount = 0;
		memcpy(beam.mat.ambient, amb, 4 * sizeof(float));
		memcpy(beam.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(beam.mat.specular, spec, 4 * sizeof(float));
		memcpy(beam.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(beam);
		beamMeshID = (int)renderer.myMeshes.size() - 1;
	}

	// ---- Smoke particle billboard quad ----
	{
		MyMesh smoke = createQuad(1.0f, 1.0f); // unit quad
		float amb[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		float diff[] = { 1.0f, 1.0f, 1.0f, 1.0f };
		float spec[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		float emis[] = { 0.0f, 0.0f, 0.0f, 0.0f };
		smoke.mat.shininess = 1.0f; smoke.mat.texCount = 1;
		memcpy(smoke.mat.ambient, amb, 4 * sizeof(float));
		memcpy(smoke.mat.diffuse, diff, 4 * sizeof(float));
		memcpy(smoke.mat.specular, spec, 4 * sizeof(float));
		memcpy(smoke.mat.emissive, emis, 4 * sizeof(float));
		renderer.myMeshes.push_back(smoke);
		smokeQuadMeshID = (int)renderer.myMeshes.size() - 1;
	}

	FlareTextureArray[0] = loadTextureDevIL("assets/crcl.tga");
	FlareTextureArray[1] = loadTextureDevIL("assets/flar.tga");
	FlareTextureArray[2] = loadTextureDevIL("assets/hxgn.tga");
	FlareTextureArray[3] = loadTextureDevIL("assets/ring.tga");
	FlareTextureArray[4] = loadTextureDevIL("assets/sun.tga");

	// ---- Load flare definition ----
	loadFlareFile(&gFlare, (FILE*)nullptr ? (char*)"assets/flare.txt" : (char*)"flare.txt");
	// Create the initial list
	initFlyingObjects();
}

// ------------------------------------------------------------
//
// Main function
//

int main(int argc, char** argv) {

	//  GLUT initialization
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL | GLUT_MULTISAMPLE);
	glClearStencil(0x0);
	glEnable(GL_STENCIL_TEST);

	glutInitContextVersion(4, 3);
	glutInitContextProfile(GLUT_CORE_PROFILE);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE | GLUT_DEBUG);

	glutInitWindowPosition(100, 100);
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
	glutMouseWheelFunc(mouseWheel);


	//	return from main loop
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	//	Init GLEW
	glewExperimental = GL_TRUE;
	glewInit();

	// some GL settings
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_MULTISAMPLE);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	printf("Vendor: %s\n", glGetString(GL_VENDOR));
	printf("Renderer: %s\n", glGetString(GL_RENDERER));
	printf("Version: %s\n", glGetString(GL_VERSION));
	printf("GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

	/* Initialization of DevIL */
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	buildScene();

	if (!renderer.setRenderMeshesShaderProg("shaders/mesh.vert", "shaders/mesh.frag") ||
		!renderer.setRenderTextShaderProg("shaders/ttf.vert", "shaders/ttf.frag"))
		return(1);

	renderer.initBatteryHUD();

	//  GLUT main loop
	glutMainLoop(); // infinite loop

	return(0);
}