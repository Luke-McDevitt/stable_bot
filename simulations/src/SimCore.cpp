// wave_sim.cpp – GPU‑accelerated 3‑D water simulation prototype
// -----------------------------------------------------------------------------
// 2025‑06‑04 – dark‑mode toggle + cube viewport fixes
//   • First checkbox switches background between white (default) and dark.
//   • Cube now renders in top‑left (20‑100 px) and picks correctly.
//   • Compile fix: removed stray ",&S.linear);" token.
// -----------------------------------------------------------------------------
// NOTE: compute shader (kFluidCS) still stubbed – this build is for UI polish.
// -----------------------------------------------------------------------------

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "SimCore.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unordered_map>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shobjidl.h>
#include <shlobj.h>
#endif

#ifdef _WIN32
#   include <windows.h>
#   include <shlobj.h>     // SHGetKnownFolderPath, SHBrowseForFolder
#   include <shellapi.h>   // ShellExecute
#   include <objbase.h>    // CoInitializeEx / CoUninitialize
// Avoid Windows annotation macros colliding with our enums
#ifdef IN
#  undef IN
#endif
#ifdef OUT
#  undef OUT
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#endif


#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/constants.hpp>   // for glm::pi, glm::half_pi



#ifdef _WIN32
extern "C" {
// force use of discrete GPU on laptops (NVIDIA & AMD)
__declspec(dllexport) unsigned long NvOptimusEnablement                  = 0x00000001;
__declspec(dllexport) int            AmdPowerXpressRequestHighPerformance = 1;
}
#endif

//---------------------------------- Compile‑time checks -------------------------
#ifndef NDEBUG
#define GL_CHECK(x) do{ x; GLenum e; while((e=glGetError())!=GL_NO_ERROR){std::cerr<<"GL err 0x"<<std::hex<<e<<std::dec<<" @"<<__LINE__<<"\n"; std::terminate();}}while(0)
#else
#define GL_CHECK(x) x
#endif

//---------------------------------- Types --------------------------------------
struct WaveParams { float angle_deg=0, height_m=0.5f, freq_hz=0.2f, lambda_m=10.f, decay_s=0; };
struct SimSettings {
    // render
    bool wire=false; int res_log2=7; float opacity=0.5f; bool dark=false;
    bool debugCube=false; // show only a big cube in the main viewport
    bool showAxes=false;  // show X/Y/Z axes (off by default)
    // domain
    float width_m=50.f, length_m=50.f, depth_m=20.f, visc=0.001f;
    // waves
    int nDir=1; bool linear=true; std::vector<WaveParams> waves;

    int cubePos = 0; // 0=Top-Left, 1=Right Side
};

struct Camera {
    float yaw=glm::radians(45.f), pitch=glm::radians(25.f), dist=80.f; glm::vec3 pan{0};
    glm::mat4 view() const {
        glm::vec3 c=pan;
        glm::vec3 eye{c.x+dist*std::cos(pitch)*std::cos(yaw), c.y+dist*std::sin(pitch), c.z+dist*std::cos(pitch)*std::sin(yaw)};
        return glm::lookAt(eye,c,{0,1,0});
    }
};

//---------------------------------- Globals ------------------------------------
static SimSettings S; static Camera Cam; static GLFWwindow* Win=nullptr; static int FBW=1280, FBH=720; static double lastX=0,lastY=0; static bool LMB=false,MMB=false,SHIFT=false;

// -------- Viewport anchors (global so we can render them) --------
struct Anchor { std::string name; bool visible; bool focused; glm::vec3 pos; };
static std::vector<Anchor> gAnchors;
static int  gFocusedAnchor = -1;
static int gSelectedAnchor = -1; // -1 = none
// Multi-select (Viewport Anchors)
static std::vector<int> gSelAnchors;


//-------- Mesh Imports ------------------------------------------------------
struct ImportedMesh {
    std::string name;
    bool        visible = true;

    // transform
    glm::vec3   pos  = glm::vec3(0);
    glm::quat   rot  = glm::quat(1,0,0,0);
    float       scale= 1.0f;

    // gpu
    GLuint      vao = 0, vbo = 0, ebo = 0;
    GLsizei     indexCount = 0;

    // local-space AABB for picking (before transform/scale)
    glm::vec3   aabbMin = glm::vec3(0);
    glm::vec3   aabbMax = glm::vec3(0);

    // --- NEW: CPU geometry for precise picking ---
    std::vector<glm::vec3> cpuVerts;     // <-- NEW
    std::vector<uint32_t>  cpuIndices;   // <-- NEW

    // --- persistence ---
    std::filesystem::path srcPath;
    std::string           savedFile;
};



static std::vector<ImportedMesh> gMeshes;
static int gSelectedMesh = -1;
// Multi-select (Bodies): indices into gMeshes (primary remains gSelectedMesh)
static std::vector<int> gSelectedMeshes; // includes primary when multi-select is active

// Multi-select (Bodies - Cubes): 0 = main debug cube, 1..N = extras
static std::vector<int> gSelCubes;

// ---- Scene Cameras (UI + drawing + move support) --------------------------
struct SceneCam {
    std::string name;
    bool        visible = true;
    bool        focused = false;
    glm::vec3   pos{0,0,0};
    glm::quat   rot = glm::quat(1,0,0,0);

    // Lens parameters
    float focal_mm    = 50.0f;   // focal length (mm)
    float aperture_f  = 2.8f;    // f-number
    float fov_deg     = 45.0f;   // vertical field of view (deg)

    // NEW: sensor size (35mm stills default)
    float sensor_w_mm = 36.0f;
    float sensor_h_mm = 24.0f;   // used for vertical FOV
};

static std::vector<SceneCam> gSceneCams;
static int gSelectedCam = -1;   // selected camera index for move & Open View
// Multi-select (Cameras)
static std::vector<int> gSelCams;

// Default Camera View size/offset when returning from pop-out to the sim window
static const ImVec2 kCamDockedSize  = ImVec2(640.0f, 400.0f); // bigger when re-docked
static const ImVec2 kCamDockedOffset= ImVec2(120.0f, 120.0f);
static const float  kCamDockedTopMargin = 56.0f;              // near top

// ---- Lens math helpers (assume 24mm sensor height by default) ----
static float gSensorHeightMM = 24.0f;

static inline float fovY_from_focal_mm(float focal_mm, float sensor_h_mm = gSensorHeightMM) {
    focal_mm = std::max(0.01f, focal_mm);
    return 2.0f * std::atan((sensor_h_mm * 0.5f) / focal_mm); // radians
}
static inline float focal_mm_from_fovY(float fovY_rad, float sensor_h_mm = gSensorHeightMM) {
    float t = std::tan(std::max(0.001f, fovY_rad) * 0.5f);
    return (sensor_h_mm * 0.5f) / t;
}


// ---- Camera View windows (state + helpers) ---------------------------------
struct CamViewWin {
    int         camIndex = -1;
    std::string title;
    bool        open      = true;
    bool        poppedOut = false;
    bool        wantPopMoveThisFrame = false; // move next Begin to outside/inside
    bool        wantResetEmbedSize = false;   // force standard size on return-to-sim
    bool        justCreated = false;          // first frame after "Open View"
    bool        wantCenterDockThisFrame = false; // force size+center on return


    // Render-to-texture resources
    GLuint      rtFBO   = 0;
    GLuint      rtColor = 0;
    GLuint      rtDepth = 0;
    int         rtW = 0, rtH = 0;      // current RT size
    int         wantW = 0, wantH = 0;  // size requested by UI this frame

};

static void destroyCamRT(CamViewWin& cv) {
    if (cv.rtDepth) { glDeleteRenderbuffers(1, &cv.rtDepth); cv.rtDepth = 0; }
    if (cv.rtColor) { glDeleteTextures(1, &cv.rtColor);      cv.rtColor = 0; }
    if (cv.rtFBO)   { glDeleteFramebuffers(1, &cv.rtFBO);    cv.rtFBO   = 0; }
    cv.rtW = cv.rtH = 0;
}

static void ensureCamRT(CamViewWin& cv, int w, int h) {
    w = std::max(4, w);
    h = std::max(4, h);
    if (cv.rtFBO && w == cv.rtW && h == cv.rtH) return;

    destroyCamRT(cv);

    glGenFramebuffers(1, &cv.rtFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, cv.rtFBO);

    glGenTextures(1, &cv.rtColor);
    glBindTexture(GL_TEXTURE_2D, cv.rtColor);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, cv.rtColor, 0);

    glGenRenderbuffers(1, &cv.rtDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, cv.rtDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, cv.rtDepth);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        destroyCamRT(cv);
        throw std::runtime_error("Camera view FBO incomplete");
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    cv.rtW = w; cv.rtH = h;
}


static std::vector<CamViewWin> gCamViews;

// Projection helper: derive vertical FOV (radians) from focal & sensor height (mm)
static inline float fovY_from_focal(float focal_mm, float sensor_h_mm = 24.0f) {
    focal_mm = std::max(0.01f, focal_mm);
    return 2.0f * std::atan((sensor_h_mm * 0.5f) / focal_mm);
}

// View helper from SceneCam
static inline glm::mat4 viewFromSceneCam(const SceneCam& c) {
    glm::vec3 f = glm::normalize(c.rot * glm::vec3(0,0,-1));
    glm::vec3 u = glm::normalize(c.rot * glm::vec3(0,1,0));
    glm::vec3 o = c.pos;
    return glm::lookAt(o, o + f, u);
}

// Small toggles used by the render lambda
static bool      gInCamSubview   = false;   // skip screen-space labels in subviews
static bool      gUseOverridePV  = false;
static glm::mat4 gOverrideP, gOverrideV;

// tiny helper: forward/up/right from quaternion
static inline void camBasis(const SceneCam& c, glm::vec3& f, glm::vec3& u, glm::vec3& r){
    f = glm::normalize(c.rot * glm::vec3(0,0,-1));
    u = glm::normalize(c.rot * glm::vec3(0,1,0));
    r = glm::normalize(glm::cross(f,u));
    u = glm::normalize(glm::cross(r,f)); // re-orthonormalize
}


// -------- Units ----------
enum class Unit { MM, CM, M, IN, FT };
static Unit gUnit = Unit::M;
static inline float unitToMeters(Unit u){
    switch(u){ case Unit::MM: return 0.001f; case Unit::CM: return 0.01f; case Unit::M: return 1.f;
               case Unit::IN: return 0.0254f; case Unit::FT: return 0.3048f; }
    return 1.f;
}
static const char* kUnitLabels[] = { "mm", "cm", "m", "in", "ft" };

// -------- Debug cube transform & selection ----------
struct CubeState {
    glm::vec3 pos{0,0,0};                 // world
    glm::quat rot = glm::quat(1,0,0,0);   // world (w,x,y,z)
    float     scale = 10.f;               // uniform scale (matches old 10x)
    bool      selected = false;
};
static CubeState gCube;
// Extra cubes created via Move ➜ Clone (committed on OK)
static std::vector<CubeState> gExtraCubes;
static int gSelectedCube = -1;   // -1: none, 0: original gCube, 1..N: gExtraCubes[i-1]

// --- Simple CPU mesh container for upload ---
struct CpuMesh { std::vector<glm::vec3> verts; std::vector<uint32_t> indices; glm::vec3 bmin{0}, bmax{0}; };

static void computeAABB(CpuMesh& m){
    if (m.verts.empty()) { m.bmin=m.bmax=glm::vec3(0); return; }
    glm::vec3 mn= m.verts[0], mx=m.verts[0];
    for (auto& v: m.verts){ mn=glm::min(mn,v); mx=glm::max(mx,v); }
    m.bmin = mn; m.bmax = mx;
}

// --- Minimal OBJ (positions + faces only) ---
static bool loadOBJ_Simple(const std::filesystem::path& p, CpuMesh& out, std::string* err){
    std::ifstream in(p); if(!in){ if(err) *err="Cannot open OBJ"; return false; }
    std::vector<glm::vec3> pos;
    std::string line;
    while (std::getline(in,line)){
        if (line.size()<2) continue;
        if (line[0]=='v' && (line[1]==' '||line[1]=='\t')){
            std::istringstream ls(line.substr(2));
            glm::vec3 v; ls>>v.x>>v.y>>v.z; pos.push_back(v);
        } else if (line[0]=='f' && (line[1]==' '||line[1]=='\t')){
            std::istringstream ls(line.substr(2));
            std::vector<int> ids; std::string tok;
            while (ls>>tok){
                // supports v, v/t, v//n, v/t/n; we only take the first int
                size_t s = tok.find('/'); if (s!=std::string::npos) tok = tok.substr(0,s);
                int i = std::stoi(tok);
                if (i<0) i = (int)pos.size()+1+i;
                ids.push_back(i-1);
            }
            // triangulate polygon
            for (size_t k=1;k+1<ids.size();++k){
                out.indices.push_back((uint32_t)out.verts.size()+0);
                out.indices.push_back((uint32_t)out.verts.size()+k);
                out.indices.push_back((uint32_t)out.verts.size()+k+1);
            }
            for (int id : ids) out.verts.push_back(pos[id]);
        }
    }
    computeAABB(out);
    return true;
}

// --- Minimal STL (ASCII + binary) ---
#pragma pack(push,1)
struct STLtri { float n[3]; float v[9]; uint16_t attr; };
#pragma pack(pop)

static bool loadSTL_Simple(const std::filesystem::path& p, CpuMesh& out, std::string* err){
    std::ifstream in(p, std::ios::binary); if (!in){ if(err)*err="Cannot open STL"; return false; }
    // detect ASCII (starts with "solid " and has "facet")
    std::string header(80,'\0'); in.read(header.data(),80);
    in.seekg(0);
    if (header.rfind("solid",0)==0){
        // try ASCII
        std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::istringstream ss(s);
        std::string w;
        std::vector<glm::vec3> tris;
        while (ss>>w){
            if (w=="vertex"){
                glm::vec3 v; ss>>v.x>>v.y>>v.z; tris.push_back(v);
            }
        }
        if (tris.size()%3==0 && tris.size()>0){
            size_t base = out.verts.size();
            out.verts.insert(out.verts.end(), tris.begin(), tris.end());
            out.indices.reserve(out.indices.size()+tris.size());
            for (size_t i=0;i<tris.size();++i) out.indices.push_back((uint32_t)(base+i));
            computeAABB(out);
            return true;
        }
        // fallthrough to binary
        in.clear(); in.seekg(0);
    }
    // binary
    in.seekg(80);
    uint32_t ntri=0; in.read(reinterpret_cast<char*>(&ntri),4);
    if (!in) { if(err)*err="Bad STL header"; return false; }
    out.verts.reserve(out.verts.size()+ntri*3);
    out.indices.reserve(out.indices.size()+ntri*3);
    for (uint32_t i=0;i<ntri;++i){
        STLtri t; in.read(reinterpret_cast<char*>(&t), sizeof(STLtri));
        if (!in){ if(err)*err="Unexpected EOF in STL"; return false; }
        uint32_t base = (uint32_t)out.verts.size();
        out.verts.emplace_back(t.v[0],t.v[1],t.v[2]);
        out.verts.emplace_back(t.v[3],t.v[4],t.v[5]);
        out.verts.emplace_back(t.v[6],t.v[7],t.v[8]);
        out.indices.push_back(base+0);
        out.indices.push_back(base+1);
        out.indices.push_back(base+2);
    }
    computeAABB(out);
    return true;
}

// --- Upload CPU mesh to GPU & create ImportedMesh ---
static void uploadMeshToGPU(const CpuMesh& m, ImportedMesh& out){
    if (!out.vao) glGenVertexArrays(1,&out.vao);
    if (!out.vbo) glGenBuffers(1,&out.vbo);
    if (!out.ebo) glGenBuffers(1,&out.ebo);
    glBindVertexArray(out.vao);
    glBindBuffer(GL_ARRAY_BUFFER, out.vbo);
    glBufferData(GL_ARRAY_BUFFER, m.verts.size()*sizeof(glm::vec3), m.verts.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, out.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m.indices.size()*sizeof(uint32_t), m.indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(glm::vec3),(void*)0);
    glBindVertexArray(0);
    out.indexCount = (GLsizei)m.indices.size();
    // Keep a CPU copy for picking
    out.cpuVerts   = m.verts;
    out.cpuIndices = m.indices;
    out.aabbMin = m.bmin;
    out.aabbMax = m.bmax;
}

static int importModelFromFile(const std::filesystem::path& file, std::string* err=nullptr){
    std::string ext = file.extension().string();
    std::transform(ext.begin(),ext.end(),ext.begin(),::tolower);

    CpuMesh cm;
    bool ok=false;
    if (ext==".obj") ok = loadOBJ_Simple(file, cm, err);
    else if (ext==".stl") ok = loadSTL_Simple(file, cm, err);
    else {
        if (err) *err = "Format not supported in this build (try OBJ or STL).";
        return -1;
    }
    if (!ok) return -1;

    ImportedMesh M;
    M.srcPath   = file;                        // remember original path
    M.savedFile = file.filename().string();    // default; may change when saving
    M.name  = file.filename().string();
    M.pos   = glm::vec3(0,0,0);
    M.rot   = glm::quat(1,0,0,0);
    M.scale = 1.0f;
    uploadMeshToGPU(cm, M);

    gMeshes.push_back(std::move(M));
    gSelectedMesh = (int)gMeshes.size()-1;
    // clear other selections
    gSelectedCube = -1;
    gCube.selected= false;
    gSelectedCam  = -1;
    gSelectedAnchor = -1;

    return gSelectedMesh;
}


// -------- Move tool ----------
enum class MoveObj { Bodies, Faces, Points, ViewportAnchors, Cameras, Components, Joints };
enum class MoveType { Free, Translate, Rotate, PointToPoint, PointToPos };

struct Pivot {
    // Stored in object LOCAL space so it rides with the body
    glm::vec3 localPos{0,0,0};        // default: local origin
    glm::mat3 localBasis = glm::mat3(1.0f); // default: object local axes
    // Derived each frame
    glm::vec3 worldPos() const { return gCube.pos + (gCube.rot * (localPos * gCube.scale)); }
    glm::mat3 worldBasis() const { return glm::mat3_cast(gCube.rot) * localBasis; }
};

struct MoveState {
    bool active = false;            // move mode shown (opened via 'm')
    MoveObj obj = MoveObj::Bodies;
    MoveType type = MoveType::Free;
    Pivot pivot;                    // current pivot (local)
    // Snapshot for cancel/OK
    glm::vec3 snapPos{0,0,0};
    glm::quat snapRot = glm::quat(1,0,0,0);

    // Which object was snapshotted at move start (so ESC restores the right one)
    enum class SnapTarget { None, Anchor, Camera, Cube, Mesh } snapTarget = SnapTarget::None;
    int  snapAnchorIdx = -1;
    int  snapCamIdx    = -1;
    int  snapCubeSel   = 0;       // 0 = main cube, >0 = gExtraCubes[index = snapCubeSel-1]
    int  snapMeshIdx   = -1;      // NEW: which mesh was snapshotted

    // Drag state
    enum Handle { None, AxisX, AxisY, AxisZ, PlaneXY, PlaneXZ, PlaneYZ, ArcX, ArcY, ArcZ } hot = None;
    bool dragging = false;
    glm::vec3 dragStartAxis;        // axis direction at drag start (world, unit)
    glm::vec3 dragStartPlaneN;      // plane normal for plane drags
    glm::vec3 dragGrabW;            // world point grabbed at start (for planes/arcs)
    glm::vec3 dragStartPw;          // axis origin at drag start (world)
    float     dragStart_tAxis = 0.f;// axis param t at drag start (closestOnAxis)
    float     dragStartAngle = 0.f; // for arcs
    // kept for other uses; not used by new axis logic
    float prevMouseX = 0.f, prevMouseY = 0.f;
    // Baseline (state at the moment a drag starts)
    glm::vec3 dragBasePos{0,0,0};
    glm::quat dragBaseRot = glm::quat(1,0,0,0);

    // --- NEW: freeze the drag frame at mouse-down ---
    glm::mat3 dragStartBasis = glm::mat3(1.0f);   // snapshot of currentPivotWorldBasis()
    glm::vec3 dragPivotStart = glm::vec3(0.0f);   // snapshot of currentPivotWorldPos()

    // UI numeric fields (debounced)
    float ui_dx = 0.f, ui_dy = 0.f, ui_dz = 0.f;   // distances in *current units*
    float ui_ax = 0.f, ui_ay = 0.f, ui_az = 0.f;   // degrees
    double lastEditTime = 0.0;                     // for 0.25s debounce
    bool   editedThisFrame = false;
    bool pickLocked = false;                                    // lock for points/faces pick
    std::unordered_map<int, glm::vec3> multiSnapPos;            // meshIdx -> original pos for cancel/deselect
    std::unordered_map<int, glm::vec3> multiSnapCubePos;        // cubeSelIdx (0 main, >0 extras) -> original pos  << ADD
    int  primaryMeshForGizmo = -1;                              // keep gizmo anchored to first mesh

    // --- Clone state (Fusion-style "Create Copy") ---
    bool      cloneChecked   = false; // checkbox in UI
    bool      cloneActive    = false; // temp clone is current move target (cube only)
    CubeState cloneCube;              // temp cube clone for preview
    int       cloneAnchorIdx = -1;    // anchor clone index (single-legacy)
    int       cloneCamIdx    = -1;    // camera clone index (single-legacy)
    int       cloneMeshIdx   = -1;    // single-mesh clone index (legacy path)

    // NEW: multi-clone bookkeeping
    std::vector<int> cloneMeshIdxs;       // meshes created this session
    std::vector<int> cloneCubeSelIdxs;    // extra cubes created this session (selection IDs)
    std::vector<int> cloneAnchorIdxs;     // NEW: anchor clones this session
    std::vector<int> cloneCamIdxs;        // NEW: camera clones this session

    // NEW: backup original selection (restored on uncheck / Cancel)
    std::vector<int> backupSelectedMeshes;
    std::vector<int> backupSelCubes;
    std::vector<int> backupSelAnchors;    // NEW
    std::vector<int> backupSelCams;       // NEW

    // NEW: per-item snapshots for viewport anchors & cameras
    std::unordered_map<int, glm::vec3> multiSnapAnchorPos; // anchor index -> pos
    std::unordered_map<int, glm::vec3> multiSnapCamPos;    // camera index -> pos
    std::unordered_map<int, glm::quat> multiSnapCamRot;    // camera index -> rot

    // last axis used for Axis drag (0=X,1=Y,2=Z, -1=none) – used for snap
    int lastAxis = -1;
    // only allow the QoL face-click snap right after an axis drag in THIS session
    bool axisSnapPending = false;

};

static MoveState gMove;

// -------- small math helpers ----------
static inline glm::vec3 screenToRay(float sx, float sy, const glm::mat4& invPV){
    float nx =  2.0f * sx / float(FBW) - 1.0f;
    float ny =  1.0f - 2.0f * sy / float(FBH);
    glm::vec4 p0 = invPV * glm::vec4(nx, ny, -1.0f, 1.0f);
    glm::vec4 p1 = invPV * glm::vec4(nx, ny,  1.0f, 1.0f);
    glm::vec3 o = glm::vec3(p0) / p0.w;
    glm::vec3 e = glm::vec3(p1) / p1.w;
    return glm::normalize(e - o); // direction only; caller supplies origin
}

static inline bool closestOnAxis(const glm::vec3& rayO, const glm::vec3& rayD,
                                 const glm::vec3& axisO, const glm::vec3& axisDir,
                                 float& tAxis, glm::vec3& hit){
    // Solve closest points between infinite lines (ray treated as line)
    glm::vec3 w0 = axisO - rayO;
    float a = 1.0f;                          // |rayD|=1
    float b = glm::dot(rayD, axisDir);
    float c = 1.0f;                          // |axisDir|=1
    float d = glm::dot(rayD, w0);
    float e = glm::dot(axisDir, w0);
    float denom = a*c - b*b;
    if (std::abs(denom) < 1e-6f) return false;
    float tRay  = (a*e - b*d) / denom;
    tAxis = (b*e - c*d) / denom;
    hit = axisO + tAxis * axisDir;
    return true;
}

static inline bool rayPlane(const glm::vec3& ro, const glm::vec3& rd,
                            const glm::vec3& p0, const glm::vec3& n,
                            glm::vec3& hit){
    float dn = glm::dot(rd, n);
    if (std::abs(dn) < 1e-6f) return false;
    float t = glm::dot(p0 - ro, n) / dn;
    if (t <= 0.f) return false;
    hit = ro + t * rd;
    return true;
}

static inline glm::mat3 faceFrame(const glm::vec3& n){
    // Build orthonormal basis with Z=n (right-handed)
    glm::vec3 z = glm::normalize(n);
    glm::vec3 h = (std::abs(z.y) < 0.99f) ? glm::vec3(0,1,0) : glm::vec3(1,0,0);
    glm::vec3 x = glm::normalize(glm::cross(h, z));
    glm::vec3 y = glm::normalize(glm::cross(z, x));
    return glm::mat3(x,y,z);
}

static inline bool anchorMode(){ return gMove.obj == MoveObj::ViewportAnchors; }
static inline bool cameraMode(){ return gMove.obj == MoveObj::Cameras; }
static inline bool haveSelection(){
    if (anchorMode()) return (!gSelAnchors.empty() || gSelectedAnchor >= 0);
    if (cameraMode()) return (!gSelCams.empty()    || gSelectedCam    >= 0);
    // Bodies: include multi-cube selection as well
    return (!gSelectedMeshes.empty() || gSelectedMesh >= 0 ||
            !gSelCubes.empty() || gSelectedCube >= 0 || gCube.selected ||
            gMove.cloneActive);
}

// --- Selection maintenance helpers (call after deleting an item) ---
static void pruneSelectionVectors() {
    // drop any invalid indices after outside changes
    auto prune = [](std::vector<int>& v, int max){
        v.erase(std::remove_if(v.begin(), v.end(),
                               [max](int i){ return i < 0 || i >= max; }),
                v.end());
    };
    prune(gSelAnchors, (int)gAnchors.size());
    if (gSelectedAnchor >= (int)gAnchors.size()) gSelectedAnchor = -1;

    prune(gSelCams, (int)gSceneCams.size());
    if (gSelectedCam >= (int)gSceneCams.size()) gSelectedCam = -1;
}



static inline int selectionCount(){
    // Always sanitize first so deleted items don't linger in the count
    pruneSelectionVectors();

    if (anchorMode()) {
        return !gSelAnchors.empty() ? (int)gSelAnchors.size()
                                    : (gSelectedAnchor >= 0 ? 1 : 0);
    }
    if (cameraMode()) {
        return !gSelCams.empty() ? (int)gSelCams.size()
                                 : (gSelectedCam >= 0 ? 1 : 0);
    }
    // Bodies
    int count = 0;
    // Meshes
    if (!gSelectedMeshes.empty())      count += (int)gSelectedMeshes.size();
    else if (gSelectedMesh >= 0)       count += 1;
    // Cubes
    if (!gSelCubes.empty())            count += (int)gSelCubes.size();
    else if (gSelectedCube >= 0 ||
             gCube.selected)           count += 1;
    // If clone preview is active, ensure we show at least 1 selected
    if (gMove.cloneActive)             count = std::max(count, 1);
    return count;
}

// Active cube getter/setter (temp clone if active, else the selected cube)
static inline const CubeState& activeCube(){
    if (gMove.cloneActive) return gMove.cloneCube;
    if (gSelectedCube == 0) return gCube;
    if (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
        return gExtraCubes[gSelectedCube - 1];
    return gCube; // fallback
}
static inline CubeState& activeCubeMut(){
    if (gMove.cloneActive) return gMove.cloneCube;
    if (gSelectedCube == 0) return gCube;
    if (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
        return gExtraCubes[gSelectedCube - 1];
    return gCube; // fallback
}

static inline glm::vec3 currentPivotWorldPos(){
    if (anchorMode()) {
        const int ai = (gSelectedAnchor >= 0) ? gSelectedAnchor
                    : (!gSelAnchors.empty()   ? gSelAnchors.front() : -1);
        return (ai >= 0 && ai < (int)gAnchors.size()) ? gAnchors[ai].pos : glm::vec3(0);
    } else if (cameraMode()) {
        const int ci = (gSelectedCam >= 0) ? gSelectedCam
                    : (!gSelCams.empty()    ? gSelCams.front()    : -1);
        if (ci >= 0 && ci < (int)gSceneCams.size()) {
            const SceneCam& C = gSceneCams[ci];
            return C.pos + (C.rot * gMove.pivot.localPos);
        }
        return glm::vec3(0);
    } else {
        // Bodies: prefer the 'primary' mesh when multi-selected (unchanged)
        int primary = -1;
        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
            primary = gSelectedMesh;
        } else if (!gSelectedMeshes.empty()) {
            if (gMove.primaryMeshForGizmo >= 0 &&
                std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), gMove.primaryMeshForGizmo) != gSelectedMeshes.end()) {
                primary = gMove.primaryMeshForGizmo;
                } else {
                    primary = gSelectedMeshes.front();
                }
        }
        if (primary >= 0 && primary < (int)gMeshes.size()) {
            const auto& M = gMeshes[primary];
            return M.pos + (M.rot * (gMove.pivot.localPos * M.scale));
        }
    }
    // Fallback: cubes when no mesh is selected (unchanged)
    const CubeState& C = activeCube();
    return C.pos + (C.rot * (gMove.pivot.localPos * C.scale));
}

static inline glm::mat3 currentPivotWorldBasis(){
    if (anchorMode()) {
        return glm::mat3(1.0f);
    } else if (cameraMode()) {
        const int ci = (gSelectedCam >= 0) ? gSelectedCam
                    : (!gSelCams.empty()    ? gSelCams.front()    : -1);
        if (ci >= 0 && ci < (int)gSceneCams.size()) {
            const SceneCam& C = gSceneCams[ci];
            return glm::mat3_cast(C.rot) * gMove.pivot.localBasis;
        }
        return glm::mat3(1.0f);
    } else {
        // Bodies: match the primary chosen in currentPivotWorldPos() (unchanged)
        int primary = -1;
        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
            primary = gSelectedMesh;
        } else if (!gSelectedMeshes.empty()) {
            if (gMove.primaryMeshForGizmo >= 0 &&
                std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), gMove.primaryMeshForGizmo) != gSelectedMeshes.end()) {
                primary = gMove.primaryMeshForGizmo;
                } else {
                    primary = gSelectedMeshes.front();
                }
        }
        if (primary >= 0 && primary < (int)gMeshes.size()) {
            const auto& M = gMeshes[primary];
            return glm::mat3_cast(M.rot) * gMove.pivot.localBasis;
        }
    }
    const CubeState& C = activeCube();
    return glm::mat3_cast(C.rot) * gMove.pivot.localBasis;
}


static glm::vec3* currentPosPtr(){
    // Cameras
    if (gMove.obj == MoveObj::Cameras) {
        int ci = (gSelectedCam >= 0) ? gSelectedCam
               : (!gSelCams.empty()   ? gSelCams.front() : -1);
        return (ci >= 0 && ci < (int)gSceneCams.size()) ? &gSceneCams[ci].pos : nullptr;
    }
    // Viewport Anchors
    if (gMove.obj == MoveObj::ViewportAnchors) {
        int ai = (gSelectedAnchor >= 0) ? gSelectedAnchor
               : (!gSelAnchors.empty()   ? gSelAnchors.front() : -1);
        return (ai >= 0 && ai < (int)gAnchors.size()) ? &gAnchors[ai].pos : nullptr;
    }
    // Bodies
    if (gMove.obj == MoveObj::Bodies) {
        // If previewing a clone, that temp clone IS the move target
        if (gMove.cloneActive && gSelectedMesh < 0)
            return &gMove.cloneCube.pos;

        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size())
            return &gMeshes[gSelectedMesh].pos;

        if      (gSelectedCube == 0) return &gCube.pos;
        else if (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
            return &gExtraCubes[gSelectedCube - 1].pos;

        return nullptr; // no implicit cube when nothing is selected
    }
    return nullptr;
}

static glm::quat* currentRotPtr(){
    // Viewport Anchors: non-rotating
    if (gMove.obj == MoveObj::ViewportAnchors) return nullptr;

    // Cameras
    if (gMove.obj == MoveObj::Cameras) {
        int ci = (gSelectedCam >= 0) ? gSelectedCam
               : (!gSelCams.empty()   ? gSelCams.front() : -1);
        return (ci >= 0 && ci < (int)gSceneCams.size()) ? &gSceneCams[ci].rot : nullptr;
    }
    // Bodies
    if (gMove.obj == MoveObj::Bodies) {
        // If previewing a clone, rotate the temp clone
        if (gMove.cloneActive && gSelectedMesh < 0)
            return &gMove.cloneCube.rot;

        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size())
            return &gMeshes[gSelectedMesh].rot;

        if      (gSelectedCube == 0) return &gCube.rot;
        else if (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
            return &gExtraCubes[gSelectedCube - 1].rot;

        return nullptr;
    }
    return nullptr;
}

// NEW: Refresh baseline snapshot for the *current* Move session when selection changes.
static void refreshMoveSnapshotForSelection()
{
    if (!gMove.active) return;

    // --- 0) If selection changed mid-move, restore anything that dropped out of selection ---
    // Do this BEFORE clearing the per-kind baseline maps so we can still use them.
    // Anchors
    if (!gMove.multiSnapAnchorPos.empty()) {
        std::vector<int> cur;
        cur.insert(cur.end(), gSelAnchors.begin(), gSelAnchors.end());
        if (gSelectedAnchor >= 0) cur.push_back(gSelectedAnchor);
        for (const auto& kv : gMove.multiSnapAnchorPos) {
            int ai = kv.first;
            if (std::find(cur.begin(), cur.end(), ai) == cur.end()) {
                if (ai >= 0 && ai < (int)gAnchors.size())
                    gAnchors[ai].pos = kv.second;
            }
        }
    }
    // Cameras (pos + rot)
    if (!gMove.multiSnapCamPos.empty() || !gMove.multiSnapCamRot.empty()) {
        std::vector<int> cur;
        cur.insert(cur.end(), gSelCams.begin(), gSelCams.end());
        if (gSelectedCam >= 0) cur.push_back(gSelectedCam);
        for (const auto& kv : gMove.multiSnapCamPos) {
            int ci = kv.first;
            if (std::find(cur.begin(), cur.end(), ci) == cur.end()) {
                if (ci >= 0 && ci < (int)gSceneCams.size()) {
                    gSceneCams[ci].pos = kv.second;
                    auto itR = gMove.multiSnapCamRot.find(ci);
                    if (itR != gMove.multiSnapCamRot.end())
                        gSceneCams[ci].rot = itR->second;
                }
            }
        }
    }
    // Meshes (pos)
    if (!gMove.multiSnapPos.empty()) {
        std::vector<int> cur;
        cur.insert(cur.end(), gSelectedMeshes.begin(), gSelectedMeshes.end());
        if (gSelectedMesh >= 0) cur.push_back(gSelectedMesh);
        for (const auto& kv : gMove.multiSnapPos) {
            int mi = kv.first;
            if (std::find(cur.begin(), cur.end(), mi) == cur.end()) {
                if (mi >= 0 && mi < (int)gMeshes.size())
                    gMeshes[mi].pos = kv.second;
            }
        }
    }
    // Cubes (pos) — map key: 0 = main cube, >0 = extra index+1
    if (!gMove.multiSnapCubePos.empty()) {
        std::vector<int> cur = gSelCubes;
        if (gSelectedCube >= 0) cur.push_back(gSelectedCube);
        if (gCube.selected)     cur.push_back(0);
        for (const auto& kv : gMove.multiSnapCubePos) {
            int ci = kv.first;
            if (std::find(cur.begin(), cur.end(), ci) == cur.end()) {
                if (ci == 0) {
                    gCube.pos = kv.second;
                } else if (ci > 0 && ci - 1 < (int)gExtraCubes.size()) {
                    gExtraCubes[ci - 1].pos = kv.second;
                }
            }
        }
    }

    // --- 1) Reset generic single-target snapshot (for numeric edit UX, etc.) ---
    gMove.snapTarget = MoveState::SnapTarget::None;
    gMove.snapPos    = glm::vec3(0.0f);
    gMove.snapRot    = glm::quat(1,0,0,0);

    // --- 2) Clear per-kind multi snapshots (we just used them; now re-baseline current selection) ---
    gMove.multiSnapPos.clear();          // meshes
    gMove.multiSnapAnchorPos.clear();    // anchors
    gMove.multiSnapCamPos.clear();       // cameras (pos)
    gMove.multiSnapCamRot.clear();       // cameras (rot)
    gMove.multiSnapCubePos.clear();      // NEW: cubes

    // --- 3) Take fresh baselines for the CURRENT selection ---
    if (anchorMode()) {
        // Anchors: snapshot all selected anchors
        for (int ai : gSelAnchors) {
            if (ai >= 0 && ai < (int)gAnchors.size())
                gMove.multiSnapAnchorPos[ai] = gAnchors[ai].pos;
        }
        // Primary for gizmo / numeric fields
        if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
            gMove.snapTarget = MoveState::SnapTarget::Anchor;
            gMove.snapPos    = gAnchors[gSelectedAnchor].pos;
        }
        return;
    }

    if (cameraMode()) {
        // Cameras: snapshot all selected cameras (pos + rot)
        for (int ci : gSelCams) {
            if (ci >= 0 && ci < (int)gSceneCams.size()) {
                gMove.multiSnapCamPos[ci] = gSceneCams[ci].pos;
                gMove.multiSnapCamRot[ci] = gSceneCams[ci].rot;
            }
        }
        // Single-select fallback
        if (gSelCams.empty() && gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
            gMove.multiSnapCamPos[gSelectedCam] = gSceneCams[gSelectedCam].pos;
            gMove.multiSnapCamRot[gSelectedCam] = gSceneCams[gSelectedCam].rot;
        }
        // Primary for gizmo / numeric fields
        if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
            gMove.snapTarget = MoveState::SnapTarget::Camera;
            gMove.snapPos    = gSceneCams[gSelectedCam].pos;
            gMove.snapRot    = gSceneCams[gSelectedCam].rot;
        }
        return;
    }

    // --- Bodies (meshes/cubes) ---
    if (gMove.obj == MoveObj::Bodies) {
        // Remember which cube index is baseline for this session (0 main, >0 extras)
        gMove.snapCubeSel = gSelectedCube;

        // 3a) Multi-mesh selection: per-mesh baselines
        if (!gSelectedMeshes.empty()) {
            for (int mi : gSelectedMeshes) {
                if (mi >= 0 && mi < (int)gMeshes.size())
                    gMove.multiSnapPos[mi] = gMeshes[mi].pos;
            }
            int primary = (gSelectedMesh >= 0) ? gSelectedMesh : gSelectedMeshes.front();
            if (primary >= 0 && primary < (int)gMeshes.size()) {
                gMove.snapTarget = MoveState::SnapTarget::Cube; // reuse path for baseline
                gMove.snapPos    = gMeshes[primary].pos;
                gMove.snapRot    = gMeshes[primary].rot;
            }
            return;
        }

        // 3b) Multi-cube selection: per-cube baselines (NEW)
        if (!gSelCubes.empty()) {
            for (int ci : gSelCubes) {
                if (ci == 0) {
                    gMove.multiSnapCubePos[0] = gCube.pos;
                } else if (ci > 0 && ci - 1 < (int)gExtraCubes.size()) {
                    gMove.multiSnapCubePos[ci] = gExtraCubes[ci - 1].pos;
                }
            }
            int primaryCi = (gSelectedCube >= 0) ? gSelectedCube : gSelCubes.front();
            const CubeState& C = (primaryCi == 0) ? gCube :
                                 ((primaryCi > 0 && primaryCi - 1 < (int)gExtraCubes.size())
                                     ? gExtraCubes[primaryCi - 1] : gCube);
            gMove.snapTarget = MoveState::SnapTarget::Cube;
            gMove.snapPos    = C.pos;
            gMove.snapRot    = C.rot;
            return;
        }

        // 3c) Single mesh
        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
            gMove.multiSnapPos[gSelectedMesh] = gMeshes[gSelectedMesh].pos;
            gMove.snapTarget = MoveState::SnapTarget::Cube;
            gMove.snapPos    = gMeshes[gSelectedMesh].pos;
            gMove.snapRot    = gMeshes[gSelectedMesh].rot;
            return;
        }

        // 3d) Single cube
        if (gSelectedCube >= 0 || gCube.selected) {
            const CubeState& C = (gSelectedCube == 0) ? gCube :
                (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
                    ? gExtraCubes[gSelectedCube - 1] : gCube;
            gMove.snapTarget = MoveState::SnapTarget::Cube;
            gMove.snapPos    = C.pos;
            gMove.snapRot    = C.rot;
            return;
        }
    }
}

static void fixSelectionAfterErase_Anchors(int erased) {
    // remove the erased index; shift all > erased down by 1
    gSelAnchors.erase(std::remove(gSelAnchors.begin(), gSelAnchors.end(), erased), gSelAnchors.end());
    for (int& j : gSelAnchors) if (j > erased) --j;
    if      (gSelectedAnchor == erased) gSelectedAnchor = -1;
    else if (gSelectedAnchor  > erased) --gSelectedAnchor;

    pruneSelectionVectors();
    if (gMove.active && anchorMode()) refreshMoveSnapshotForSelection();
}

static void fixSelectionAfterErase_Cams(int erased) {
    gSelCams.erase(std::remove(gSelCams.begin(), gSelCams.end(), erased), gSelCams.end());
    for (int& j : gSelCams) if (j > erased) --j;
    if      (gSelectedCam == erased) gSelectedCam = -1;
    else if (gSelectedCam  > erased) --gSelectedCam;

    pruneSelectionVectors();
    if (gMove.active && cameraMode()) refreshMoveSnapshotForSelection();
}


// Apply a translation to the current selection.
// Bodies: apply to all selected meshes (multi-select). Cubes/cameras/anchors: single target.
static void applyTranslateToSelection(const glm::vec3& deltaW)
{
    if (gMove.obj == MoveObj::Bodies) {
        // Prefer meshes if any are selected
        if (!gSelectedMeshes.empty()) {
            for (int idx : gSelectedMeshes) {
                if (idx >= 0 && idx < (int)gMeshes.size())
                    gMeshes[idx].pos += deltaW;
            }
            return;
        }
        if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
            gMeshes[gSelectedMesh].pos += deltaW;
            return;
        }
        // Multi-select: cubes
        if (!gSelCubes.empty()) {
            for (int ci : gSelCubes) {
                if (ci == 0) {
                    gCube.pos += deltaW;
                } else if (ci > 0 && ci - 1 < (int)gExtraCubes.size()) {
                    gExtraCubes[ci - 1].pos += deltaW;
                }
            }
            return;
        }

        if (gMove.cloneActive) { gMove.cloneCube.pos += deltaW; return; }
        // Fall back to cubes
        if (gSelectedCube == 0) { gCube.pos += deltaW; return; }
        if (gSelectedCube > 0 && gSelectedCube-1 < (int)gExtraCubes.size()) {
            gExtraCubes[gSelectedCube-1].pos += deltaW; return;
        }
        return;
    }

    // Non-body targets (multi for cams/anchors, else single)
    if (gMove.obj == MoveObj::Cameras) {
        if (!gSelCams.empty()) {
            for (int ci : gSelCams)
                if (ci >= 0 && ci < (int)gSceneCams.size())
                    gSceneCams[ci].pos += deltaW;
        } else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
            gSceneCams[gSelectedCam].pos += deltaW;
        }
    } else if (gMove.obj == MoveObj::ViewportAnchors) {
        if (!gSelAnchors.empty()) {
            for (int ai : gSelAnchors)
                if (ai >= 0 && ai < (int)gAnchors.size())
                    gAnchors[ai].pos += deltaW;
        } else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
            gAnchors[gSelectedAnchor].pos += deltaW;
        }
    } else {
        if (glm::vec3* P = currentPosPtr())
            *P += deltaW;
    }

}



struct SnapPlaneHit { glm::vec3 p0; glm::vec3 n; float t; bool ok=false; };

// Find nearest plane hit with normal ~ parallel to `ax`.
// Considers tank walls/top/bottom and any visible cubes (original, extras, temp clone).
static SnapPlaneHit findSnapPlane(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& ax)
{
    const float eps = 1e-6f;
    auto push = [&](SnapPlaneHit& best, const glm::vec3& p0, const glm::vec3& n){
        float dn = glm::dot(rd, n); if (std::abs(dn) < eps) return;
        float t = glm::dot(p0 - ro, n) / dn; if (t <= 0.f) return;
        if (!best.ok || t < best.t) { best = {p0, glm::normalize(n), t, true}; }
    };

    SnapPlaneHit best;

    // ---- Tank planes (bounded) ----
    float w = 0.5f * S.width_m, l = 0.5f * S.length_m, yTop = 0.f, yBot = -S.depth_m;

    auto hitPlaneBounded = [&](const glm::vec3& p0, const glm::vec3& n,
                               auto inside)->void{
        float dn = glm::dot(rd, n); if (std::abs(dn) < eps) return;
        float t  = glm::dot(p0 - ro, n) / dn; if (t <= 0.f) return;
        glm::vec3 h = ro + t*rd; if (!inside(h)) return;
        if (std::abs(glm::dot(glm::normalize(n), glm::normalize(ax))) < 0.8f) return;
        if (!best.ok || t < best.t) best = {p0, glm::normalize(n), t, true};
    };

    // X walls
    hitPlaneBounded({+w,0,0}, {+1,0,0}, [&](const glm::vec3& h){ return h.z>=-l-1e-4f && h.z<=+l+1e-4f && h.y>=yBot-1e-4f && h.y<=yTop+1e-4f; });
    hitPlaneBounded({-w,0,0}, {-1,0,0}, [&](const glm::vec3& h){ return h.z>=-l-1e-4f && h.z<=+l+1e-4f && h.y>=yBot-1e-4f && h.y<=yTop+1e-4f; });
    // Z walls
    hitPlaneBounded({0,0,+l}, {0,0,+1}, [&](const glm::vec3& h){ return h.x>=-w-1e-4f && h.x<=+w+1e-4f && h.y>=yBot-1e-4f && h.y<=yTop+1e-4f; });
    hitPlaneBounded({0,0,-l}, {0,0,-1}, [&](const glm::vec3& h){ return h.x>=-w-1e-4f && h.x<=+w+1e-4f && h.y>=yBot-1e-4f && h.y<=yTop+1e-4f; });
    // Top/bottom (for Y axis snaps)
    hitPlaneBounded({0,yTop,0}, {0,+1,0}, [&](const glm::vec3& h){ return h.x>=-w-1e-4f && h.x<=+w+1e-4f && h.z>=-l-1e-4f && h.z<=+l+1e-4f; });
    hitPlaneBounded({0,yBot,0}, {0,-1,0}, [&](const glm::vec3& h){ return h.x>=-w-1e-4f && h.x<=+w+1e-4f && h.z>=-l-1e-4f && h.z<=+l+1e-4f; });

    // ---- Debug cube faces (if visible) ----
    auto testCube = [&](const CubeState& C){
        glm::mat4 M = glm::translate(glm::mat4(1.f), C.pos)
                    * glm::mat4_cast(C.rot)
                    * glm::scale(glm::mat4(1.f), glm::vec3(C.scale));
        glm::mat4 invM = glm::inverse(M);
        glm::vec3 rO = glm::vec3(invM * glm::vec4(ro,1));
        glm::vec3 rD = glm::normalize(glm::vec3(invM * glm::vec4(rd,0)));
        glm::vec3 tmin = (-0.5f - rO) / rD;
        glm::vec3 tmax = ( 0.5f - rO) / rD;
        glm::vec3 t0 = glm::min(tmin,tmax), t1 = glm::max(tmin,tmax);
        float tEnter = std::max({t0.x,t0.y,t0.z});
        float tExit  = std::min({t1.x,t1.y,t1.z});
        if (tEnter<=tExit && tExit>0.f) {
            glm::vec3 pLocal = rO + tEnter * rD;
            glm::vec3 nLocal(0);
            glm::vec3 ap = glm::abs(pLocal);
            if (ap.x>ap.y && ap.x>ap.z) nLocal = glm::vec3((pLocal.x>0)?+1:-1,0,0);
            else if (ap.y>ap.x && ap.y>ap.z) nLocal = glm::vec3(0,(pLocal.y>0)?+1:-1,0);
            else nLocal = glm::vec3(0,0,(pLocal.z>0)?+1:-1);
            glm::vec3 nWorld = glm::mat3(glm::transpose(glm::inverse(glm::mat3(M)))) * nLocal;
            nWorld = glm::normalize(nWorld);
            if (std::abs(glm::dot(nWorld, glm::normalize(ax))) >= 0.8f) {
                glm::vec3 p0 = glm::vec3(M * glm::vec4(0.5f*nLocal,1));
                push(best, p0, nWorld);
            }
        }
    };

    if (S.debugCube) {
        testCube(gCube);
        for (const auto& C : gExtraCubes) testCube(C);
        if (gMove.cloneActive) testCube(gMove.cloneCube);
    }
    return best;
}

//--------------------------- Save/Config (paths + persistence) -----------------
namespace cfg {

struct AppPaths {
    std::filesystem::path documentsDir;     // e.g., C:\Users\<you>\Documents
    std::filesystem::path baseDir;          // Documents\MIL_McDevitt_2025
    std::filesystem::path defaultSavesDir;  // ...\Saves
    std::filesystem::path configDir;        // ...\config
    std::filesystem::path configFile;       // ...\config\settings.csv
    std::filesystem::path currentSavesDir;  // what we actually use
};
static AppPaths G;

#ifdef _WIN32
static std::filesystem::path getDocumentsFolder() {
    PWSTR pathW = nullptr;
    std::filesystem::path out;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_Documents, 0, NULL, &pathW))) {
        out = std::filesystem::path(pathW);
        CoTaskMemFree(pathW);
    }
    if (out.empty()) {
        // Fallback: %USERPROFILE%\Documents
        char* prof = std::getenv("USERPROFILE");
        out = prof ? (std::filesystem::path(prof) / "Documents") : std::filesystem::path(".");
    }
    return out;
}
#else
static std::filesystem::path getDocumentsFolder() {
    char* home = std::getenv("HOME");
    return home ? (std::filesystem::path(home) / "Documents") : std::filesystem::path(".");
}
#endif

static void ensureDirs() {
    std::error_code ec;
    std::filesystem::create_directories(G.configDir, ec);
    std::filesystem::create_directories(G.defaultSavesDir, ec);
    if (!G.currentSavesDir.empty())
        std::filesystem::create_directories(G.currentSavesDir, ec);
}

static std::string trim(std::string s) {
    auto issp = [](unsigned char c){ return std::isspace(c); };
    while (!s.empty() && issp((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && issp((unsigned char)s.back()))  s.pop_back();
    if (s.size() >= 2 && ((s.front()=='"' && s.back()=='"') || (s.front()=='\'' && s.back()=='\''))) {
        s = s.substr(1, s.size()-2);
    }
    return s;
}

static void saveConfig(bool useDefault) {
    std::error_code ec;
    std::filesystem::create_directories(G.configDir, ec);

    std::ofstream f(G.configFile, std::ios::trunc);
    if (!f) return;
    // CSV with header, ready for future options
    f << "key,value\n";
    f << "version,1\n";
    f << "use_default," << (useDefault ? "true" : "false") << "\n";
    f << "save_dir," << (useDefault ? "" : G.currentSavesDir.string()) << "\n";
    // NEW: persist UI prefs
    f << "dark_mode," << (S.dark ? "true" : "false") << "\n";
    f << "wireframe," << (S.wire ? "true" : "false") << "\n";
    f.flush();
}

static void loadConfig() {
    bool useDefault = true;
    std::string saveDirStr;

    // Defaults for UI prefs (in case keys are missing on old configs)
    bool darkMode  = false;
    bool wireframe = false;

    auto toLower = [](std::string s){
        for (auto& c : s) c = (char)std::tolower((unsigned char)c);
        return s;
    };
    auto parseBoolTrue = [&](const std::string& v0){
        std::string v = toLower(v0);
        return (v=="true" || v=="1" || v=="yes");
    };

    if (std::filesystem::exists(G.configFile)) {
        std::ifstream f(G.configFile);
        std::string line;
        while (std::getline(f, line)) {
            if (line.empty() || line[0]=='#') continue;
            auto comma = line.find(',');
            if (comma == std::string::npos) continue;
            std::string key = trim(line.substr(0, comma));
            std::string val = trim(line.substr(comma+1));

            if (key == "use_default") {
                std::string v = toLower(val);
                useDefault = !(v=="false" || v=="0" || v=="no");
            } else if (key == "save_dir") {
                saveDirStr = val;
            } else if (key == "dark_mode") {              // NEW
                darkMode = parseBoolTrue(val);
            } else if (key == "wireframe") {              // NEW
                wireframe = parseBoolTrue(val);
            }
        }
    } else {
        // First run: write defaults
        saveConfig(true);
    }

    if (!useDefault && !saveDirStr.empty()) {
        G.currentSavesDir = std::filesystem::path(saveDirStr);
    } else {
        G.currentSavesDir = G.defaultSavesDir;
    }

    // Apply UI prefs
    S.dark = darkMode;
    S.wire = wireframe;
}

#ifdef _WIN32
static bool openDirectoryInExplorer(const std::filesystem::path& p) {
    if (p.empty()) return false;
    std::wstring target = p.wstring();
    HINSTANCE h = ShellExecuteW(NULL, L"open", L"explorer.exe", target.c_str(), NULL, SW_SHOWNORMAL);
    return (INT_PTR)h > 32;
}

static std::filesystem::path chooseDirectoryDialog(const std::filesystem::path& startIn = {}) {
    CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
    BROWSEINFOW bi{};
    wchar_t display[MAX_PATH]{};
    bi.hwndOwner = NULL;
    bi.pszDisplayName = display;
    bi.lpszTitle = L"Select a folder for Saves";
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE | BIF_EDITBOX;

    PIDLIST_ABSOLUTE pidl = SHBrowseForFolderW(&bi);
    std::filesystem::path result;
    if (pidl) {
        wchar_t buf[MAX_PATH]{};
        if (SHGetPathFromIDListW(pidl, buf)) result = std::filesystem::path(buf);
        CoTaskMemFree(pidl);
    }
    CoUninitialize();
    return result;
}
#else
static bool openDirectoryInExplorer(const std::filesystem::path&) { return false; }
static std::filesystem::path chooseDirectoryDialog(const std::filesystem::path& = {}) { return {}; }
#endif

static void initSaveSystem() {
    G.documentsDir     = getDocumentsFolder();
    G.baseDir          = G.documentsDir / "MIL_McDevitt_2025";
    G.defaultSavesDir  = G.baseDir     / "Saves";
    G.configDir        = G.baseDir     / "config";
    G.configFile       = G.configDir   / "settings.csv";
    G.currentSavesDir  = G.defaultSavesDir;  // temporary; loadConfig may override

    ensureDirs();   // make base/config/default on first run
    loadConfig();   // may set currentSavesDir to override
    ensureDirs();   // ensure chosen currentSavesDir exists
}

} // namespace cfg

//---------------------------------- Shaders ------------------------------------
static const char* SurfVS = R"(#version 450 core
#define MAX_WAVES 32
layout(location=0) in vec3 p;
uniform mat4 uMVP; uniform int uN; uniform vec4 uDirAmp[MAX_WAVES]; uniform vec2 uWaveLen[MAX_WAVES]; uniform float uTime;
void main(){ float y=0.0; for(int i=0;i<uN;++i){ vec2 d=uDirAmp[i].xy; float A=uDirAmp[i].z; float w=uDirAmp[i].w; float k=uWaveLen[i].x; float phi=uWaveLen[i].y; float theta=k*dot(d,p.xz)-w*uTime+phi; y+=A*sin(theta);} vec3 pos=p; pos.y+=y; gl_Position=uMVP*vec4(pos,1);} )";
static const char* SurfFS = R"(#version 450 core
out vec4 f; uniform float uAlp; void main(){ f=vec4(0.0,0.6,0.9,uAlp);} )";
static const char* CubeVS = R"(#version 450 core
layout(location = 0) in vec3 p;
uniform mat4 uMVP;
void main() { gl_Position = uMVP * vec4(p, 1.0); })";
static const char* CubeFS = R"(#version 450 core
uniform vec4 uColor;
out vec4 f;
void main() { f = uColor; })";

//---------------------------------- Globals --------------------------------

// ---- Cube overlay config & state ----
static int   gCubeX = 20;                // updated each frame
static int   gCubeY = 20;                // (top-left margin)
static const int CubePx = 120;

static bool   cubeDragging = false;
static double cubeDragLastX = 0.0, cubeDragLastY = 0.0;
// CLICK vs DRAG disambiguation for the nav cube
static bool   cubePendingClick = false;
static double cubePressX = 0.0, cubePressY = 0.0;
static int    cubePressFace = -1;   // face under the cursor at press (if any)

// "Home" button (fit view) — UI geometry
static const int HomeBtnSize = 28;
static const int HomeBtnPad  = 4;

// Settings window info captured each frame inside settingsUI()
static float gSettingsBottomY   = 0.0f;
static bool  gSettingsCollapsed = false;
static float gRightStackBottomY = 0.0f;   // bottom of right-stack UI (Settings + Anchor)
static bool  gAnchorCollapsed   = true;   // current collapsed state of Anchor window
// --- Top-left bar (File) geometry so we can push the nav cube down
static float gTopLeftBarBottomY = 0.0f;

// --- Scene Save/Load UI state ---
static bool        gOpenSaveModal = false;
static bool        gOpenLoadModal = false;
static char        gSaveNameBuf[128] = {0};
static std::string gSceneIOMessage;


// Build the exact same MVP used for drawing the cube
static inline glm::mat4 makeCubeMVP()
{
    glm::mat4 cubeP = glm::ortho(-1.f, 1.f, -1.f, 1.f, 0.1f, 6.f);
    glm::mat4 cubeR = glm::mat4(glm::mat3(Cam.view()));          // yaw/pitch only
    glm::mat4 cubeS = glm::scale(glm::mat4(1.f), glm::vec3(0.75f));
    glm::mat4 cubeT = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -1.8f));
    return cubeP * cubeT * cubeR * cubeS;  // order matters
}

// Home button rect in screen coords
// Home button rectangle in framebuffer pixels, positioned beside the cube.
// Left-anchored cube: Home to the RIGHT of the cube.
// Right-anchored cube: Home to the LEFT of the cube (keeps it on-screen).
// Convert framebuffer -> screen helpers
static inline float FB2S_X(float x){ return x / ImGui::GetIO().DisplayFramebufferScale.x; }
static inline float FB2S_Y(float y){ return y / ImGui::GetIO().DisplayFramebufferScale.y; }

static inline bool inCubeRect(double mx, double my)
{
    ImGuiIO& io = ImGui::GetIO();
    const float x0 = FB2S_X((float)gCubeX);
    const float y0 = FB2S_Y((float)gCubeY);
    const float x1 = x0 + FB2S_X((float)CubePx);
    const float y1 = y0 + FB2S_Y((float)CubePx);
    return (mx >= x0 && mx <= x1 && my >= y0 && my <= y1);
}


// Home button rect in *main-viewport absolute* ImGui coordinates,
// aligned to the TOP of the cube, placed beside it.
static void getHomeBtnRect(float& x0, float& y0, float& x1, float& y1)
{
    const float size = 26.0f;   // image + hitbox size (window px)
    const float gap  = 10.0f;

    // Cube rect in window pixels derived from framebuffer-locked gCubeX/Y
    const float cubeX_w = FB2S_X((float)gCubeX);
    const float cubeY_w = FB2S_Y((float)gCubeY);
    const float cubeW_w = FB2S_X((float)CubePx);

    if (S.cubePos == 0) {
        // cube on LEFT -> Home to the RIGHT
        x0 = cubeX_w + cubeW_w + gap;
        x1 = x0 + size;
    } else {
        // cube on RIGHT -> Home to the LEFT
        x1 = cubeX_w - gap;
        x0 = x1 - size;
    }

    // lock to TOP edge of the cube
    y0 = cubeY_w;
    y1 = y0 + size;

    // Convert to absolute coordinates of the main ImGui viewport
    ImVec2 vp = ImGui::GetMainViewport()->Pos;
    x0 += vp.x; x1 += vp.x;
    y0 += vp.y; y1 += vp.y;
}

// Hover test in the *same* coordinate space as drawing
static inline bool inHomeBtnRect()
{
    float x0,y0,x1,y1; getHomeBtnRect(x0,y0,x1,y1);
    ImVec2 mp = ImGui::GetIO().MousePos; // absolute ImGui coords
    return (mp.x >= x0 && mp.x <= x1 && mp.y >= y0 && mp.y <= y1);
}

// Place nav cube using framebuffer pixels (top/left origin for our UI logic).
// Left: below the File bar. Right: below the right-side stack.
// Lock nav cube to the simulation framebuffer (not ImGui screen).
static void computeCubePlacement()
{
    ImGuiIO& io = ImGui::GetIO();
    const float sx = io.DisplayFramebufferScale.x;   // screen->FB scale X
    const float sy = io.DisplayFramebufferScale.y;   // screen->FB scale Y

    // Side/top margins in SCREEN px -> convert to FB px
    const int marginFB_X   = (int)std::round(18.0f * sx);   // side margin
    const int minTopFB_Y   = (int)std::round(56.0f * sy);   // minimum top margin
    const int gapUnderUIFB = (int)std::round(10.0f * sy);   // extra gap below UI
    const int bottomPadFB  = (int)std::round(16.0f * sy);   // keep off very bottom

    // Convert the measured ImGui window bottoms (absolute screen px)
    // to MAIN VIEWPORT-LOCAL px, then to framebuffer px.
    const ImVec2 vpPos = ImGui::GetMainViewport()->Pos;

    // Left stack = File bar bottom; Right stack = Settings/Anchors/Cameras bottom
    const float fileBarBottom_localY   = std::max(0.0f, gTopLeftBarBottomY  - vpPos.y);
    const float rightStackBottom_localY= std::max(0.0f, gRightStackBottomY - vpPos.y);

    // Convert those LOCAL screen px to FB px
    const int fileBarBottomFB_Y    = (int)std::round((fileBarBottom_localY   * sy));
    const int rightStackBottomFB_Y = (int)std::round((rightStackBottom_localY* sy));

    // Base X from side, Y from either the live UI bottom (plus a gap) or the minimum top margin
    if (S.cubePos == 0) {
        // LEFT
        gCubeX = marginFB_X;
        int yFromUI = fileBarBottomFB_Y + gapUnderUIFB;
        gCubeY = std::max(minTopFB_Y, yFromUI);
    } else {
        // RIGHT
        gCubeX = FBW - marginFB_X - CubePx;
        int yFromUI = rightStackBottomFB_Y + gapUnderUIFB;
        gCubeY = std::max(minTopFB_Y, yFromUI);
    }

    // Clamp vertically inside the framebuffer
    if (gCubeY > FBH - CubePx - bottomPadFB)
        gCubeY = FBH - CubePx - bottomPadFB;
    if (gCubeY < minTopFB_Y)
        gCubeY = minTopFB_Y;
}


// Map (axis,sign) -> face index matching face buffer order:
// 0: -Z, 1: +Z, 2: +X, 3: -X, 4: +Y, 5: -Y
static inline int axisSgnToFace(int axis, float sgn)
{
    if (axis == 0) return (sgn > 0) ? 2 : 3; // +X : -X
    if (axis == 1) return (sgn > 0) ? 4 : 5; // +Y : -Y
    /* axis == 2 */ return (sgn > 0) ? 1 : 0; // +Z : -Z
}

// Return face index (0..5) or -1 if nothing; respects occlusion (frontmost hit)
static int pickCubeFaceAt(double mx, double my)
{
    if (!inCubeRect(mx,my)) return -1;

    // mouse -> NDC inside the cube viewport (origin top-left in window)
    float nx = float(mx - gCubeX) / float(CubePx) * 2.0f - 1.0f;
    float ny = float(CubePx - (my - gCubeY)) / float(CubePx) * 2.0f - 1.0f;

    glm::mat4 cubeMVP = makeCubeMVP();
    glm::mat4 inv     = glm::inverse(cubeMVP);

    glm::vec3 pNear = glm::vec3(inv * glm::vec4(nx, ny, -1.0f, 1.0f));
    glm::vec3 pFar  = glm::vec3(inv * glm::vec4(nx, ny,  1.0f, 1.0f));
    glm::vec3 rayO  = pNear;
    glm::vec3 rayD  = glm::normalize(pFar - pNear);

    const float h = 0.375f;
    const float eps = 1e-5f;

    float bestT = 1e9f; int bestAxis = -1; float bestSgn = 0.f;

    for (int axis = 0; axis < 3; ++axis) {
        float o = rayO[axis], d = rayD[axis];
        if (std::fabs(d) < eps) continue; // ray || slab

        for (float s : { -1.f, 1.f }) {
            float t = (s * h - o) / d;     // plane hit
            if (t <= 0.0f || t >= bestT) continue;

            glm::vec3 p = rayO + t * rayD; // intersection point
            int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
            if (std::fabs(p[a1]) <= h + 1e-4f && std::fabs(p[a2]) <= h + 1e-4f) {
                bestT = t; bestAxis = axis; bestSgn = s;
            }
        }
    }
    return (bestAxis < 0) ? -1 : axisSgnToFace(bestAxis, bestSgn);
}

// Snap camera orientation for a given cube face
static inline void applyFaceSnap(int face)
{
    switch (face) {
        case 2: Cam.yaw = 0.0f;                   Cam.pitch = 0.0f; break;                  // +X (right)
        case 3: Cam.yaw = glm::pi<float>();       Cam.pitch = 0.0f; break;                  // -X (left)
        case 1: Cam.yaw =  glm::half_pi<float>(); Cam.pitch = 0.0f; break;                  // +Z (front)
        case 0: Cam.yaw = -glm::half_pi<float>(); Cam.pitch = 0.0f; break;                  // -Z (back)
        case 4: Cam.yaw = 0.0f; Cam.pitch =  glm::half_pi<float>();  break;                 // +Y (top)
        case 5: Cam.yaw = 0.0f; Cam.pitch = -glm::half_pi<float>();  break;                 // -Y (bottom)
        default: break;
    }
}

// Fit whatever is currently visible (debug cube OR water tank), plus visible anchors.
static inline void frameAll()
{
    // Scene AABB in world-space
    glm::vec3 bmin( 1e9f,  1e9f,  1e9f);
    glm::vec3 bmax(-1e9f, -1e9f, -1e9f);

    auto expandPt = [&](const glm::vec3& p){
        bmin = glm::min(bmin, p);
        bmax = glm::max(bmax, p);
    };
    auto expandSphere = [&](const glm::vec3& c, float r){
        expandPt(c - glm::vec3(r));
        expandPt(c + glm::vec3(r));
    };

    if (S.debugCube) {
        // Debug cube visible: frame it
        // (uniform scale, any rotation -> use bounding sphere)
        float r = gCube.scale * 0.5f * std::sqrt(3.0f);
        expandSphere(gCube.pos, r);
    } else {
        // Water tank visible: frame domain (width/length/depth)
        float w = std::max(0.001f, S.width_m)  * 0.5f;
        float l = std::max(0.001f, S.length_m) * 0.5f;
        float d = std::max(0.0f,   S.depth_m);

        // Sum of wave amplitudes as an upper bound for crests
        float crest = 0.f;
        for (const auto& wv : S.waves) crest += std::abs(wv.height_m);

        // Domain AABB (y from -depth to +crest)
        expandPt(glm::vec3(-w, -d, -l));
        expandPt(glm::vec3( w,  std::max(0.0f, crest),  l));
    }

    // Include visible anchors (same radius you use when drawing them)
    if (!gAnchors.empty()) {
        float Ranc = 0.02f * std::max({ S.width_m, S.length_m, S.depth_m });
        Ranc = std::clamp(Ranc, 0.08f, 1.5f);
        for (const auto& a : gAnchors) {
            if (!a.visible) continue;
            expandSphere(a.pos, Ranc);
        }
    }

    // Fallback if nothing expanded (shouldn't happen)
    if (!(bmin.x < bmax.x && bmin.y < bmax.y && bmin.z < bmax.z)) {
        bmin = glm::vec3(-1,-1,-1);
        bmax = glm::vec3( 1, 1, 1);
    }

    // Center + bounding sphere radius
    glm::vec3 c   = 0.5f * (bmin + bmax);
    glm::vec3 ext = 0.5f * (bmax - bmin);
    float R = std::max( glm::length(ext), 0.001f );

    // Fit to current perspective (fovy=45°, aspect from framebuffer)
    float fovy   = glm::radians(45.f);
    float aspect = (FBH > 0) ? float(FBW)/float(FBH) : 1.7778f;
    float tan_v  = std::tan(fovy * 0.5f);
    float tan_h  = tan_v * aspect;
    float d_v    = R / tan_v;
    float d_h    = R / tan_h;
    float d_req  = std::max(d_v, d_h) * 1.15f; // a little margin

    Cam.pan  = c;                          // center on scene
    Cam.dist = std::clamp(d_req, 1.0f, 1800.0f); // keep inside far plane (set to taste)
}

//---------------------------------- GL objects ---------------------------------
static GLuint progSurf=0, progCube=0;
static GLuint vaoSurf=0,vboSurf=0,eboSurf=0;   // ← keep
static size_t idxCount=0;
/* cube: one VAO/VBO for wire-edges, one for solid faces  */
static GLuint vaoCubeEdge = 0, vboCubeEdge = 0;
static GLuint vaoCubeFill = 0, vboCubeFill = 0;

/* axes: one VAO/VBO for 3 colored lines (X,Y,Z) */
static GLuint vaoAxes = 0, vboAxes = 0;

/* small sphere for anchor markers */
static GLuint vaoSphere = 0, vboSphere = 0, eboSphere = 0;
static GLsizei sphereIndexCount = 0;

static GLuint vaoSides=0, vboSides=0, eboSides=0;
static GLsizei sideIndexCount = 0;
static int   cubeHoverFace = -1;     // 0-5 if mouse over, -1 otherwise

//---------------------------------- Helpers ------------------------------------
static GLuint comp(GLenum t,const char* s){ GLuint id=glCreateShader(t); glShaderSource(id,1,&s,nullptr); glCompileShader(id); GLint ok; glGetShaderiv(id,GL_COMPILE_STATUS,&ok); if(!ok){ char log[512]; glGetShaderInfoLog(id,512,nullptr,log); throw std::runtime_error(log);} return id; }
static GLuint link(std::initializer_list<GLuint> sh){ GLuint p=glCreateProgram(); for(auto s:sh) glAttachShader(p,s); glLinkProgram(p); GLint ok; glGetProgramiv(p,GL_LINK_STATUS,&ok); if(!ok){ char log[512]; glGetProgramInfoLog(p,512,nullptr,log); throw std::runtime_error(log);} for(auto s:sh) glDeleteShader(s); return p; }

//---------------------------------- Mesh builders -------------------------------
static void buildSurface(){ int n=1<<S.res_log2; std::vector<glm::vec3> v(n*n); std::vector<uint32_t> idx; idx.reserve((n-1)*(n-1)*6);
    for(int z=0;z<n;++z) for(int x=0;x<n;++x){ float fx=float(x)/(n-1), fz=float(z)/(n-1); float px=(fx-0.5f)*S.width_m; float pz=(fz-0.5f)*S.length_m; v[z*n+x]={px,0,pz}; if(x<n-1&&z<n-1){ uint32_t a=z*n+x,b=a+1,c=a+n,d=c+1; idx.insert(idx.end(),{a,b,c,b,d,c}); }}
    idxCount=idx.size(); if(!vaoSurf) glGenVertexArrays(1,&vaoSurf); if(!vboSurf) glGenBuffers(1,&vboSurf); if(!eboSurf) glGenBuffers(1,&eboSurf);
    glBindVertexArray(vaoSurf); // side walls
    glBindBuffer(GL_ARRAY_BUFFER,vboSurf); glBufferData(GL_ARRAY_BUFFER,v.size()*sizeof(glm::vec3),v.data(),GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,eboSurf);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,idx.size()*sizeof(uint32_t),idx.data(),GL_STATIC_DRAW); glBindVertexArray(0);
    // ---------- build side walls ----------
    std::vector<glm::vec3> vs;
    std::vector<uint32_t>  isw;
    float d = -S.depth_m;                               // goes down

    auto quad = [&](glm::vec3 a, glm::vec3 b,
                    glm::vec3 c, glm::vec3 d_){
        uint32_t base = vs.size();
        vs.insert(vs.end(), {a,b,c,d_});
        isw.insert(isw.end(),
                   {base,base+1,base+2,  base,base+2,base+3});
    };

    float w = 0.5f * S.width_m;
    float l = 0.5f * S.length_m;

    // +X, –X, +Z, –Z
    quad({ w,0,-l},{ w,0, l},{ w,d, l},{ w,d,-l});
    quad({-w,0, l},{-w,0,-l},{-w,d,-l},{-w,d, l});
    quad({-w,0, l},{ w,0, l},{ w,d, l},{-w,d, l});
    quad({ w,0,-l},{-w,0,-l},{-w,d,-l},{ w,d,-l});

    sideIndexCount = isw.size();

    if(!vaoSides) glGenVertexArrays(1,&vaoSides);
    if(!vboSides) glGenBuffers(1,&vboSides);
    if(!eboSides) glGenBuffers(1,&eboSides);

    glBindVertexArray(vaoSides);
    glBindBuffer(GL_ARRAY_BUFFER,  vboSides);
    glBufferData(GL_ARRAY_BUFFER,
                 vs.size()*sizeof(glm::vec3), vs.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboSides);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 isw.size()*sizeof(uint32_t), isw.data(), GL_STATIC_DRAW);

}
static void buildCube()
{
    /* ---------------- 8 unit‑cube corners ---------------- */
    const glm::vec3 v[8] = {
        {-1,-1,-1},{ 1,-1,-1},{ 1, 1,-1},{-1, 1,-1},
        {-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},{-1, 1, 1}
    };

    /* ---------- edge list (24 vertices, GL_LINES) -------- */
    const uint32_t e[24] = {
        0,1, 1,2, 2,3, 3,0,          /* bottom */
        4,5, 5,6, 6,7, 7,4,          /* top    */
        0,4, 1,5, 2,6, 3,7           /* pillars*/
    };
    std::vector<glm::vec3> edgeVerts;
    edgeVerts.reserve(24);
    for (uint32_t i: e) edgeVerts.push_back(0.5f * v[i]);   // half‑size

    /* ---------- face list (36 vertices, GL_TRIANGLES) ---- */
    const uint32_t f[36] = {
        0,1,2, 2,3,0,    // −Z
        4,5,6, 6,7,4,    // +Z
        1,5,6, 6,2,1,    // +X
        0,4,7, 7,3,0,    // −X
        3,2,6, 6,7,3,    // +Y (top)
        0,1,5, 5,4,0     // −Y (bottom)
    };
    std::vector<glm::vec3> faceVerts;
    faceVerts.reserve(36);
    for (uint32_t i: f) faceVerts.push_back(0.5f * v[i]);

    /* ----- create / fill the two VAOs -------------------- */
    if (!vaoCubeEdge) glGenVertexArrays(1,&vaoCubeEdge);
    if (!vboCubeEdge) glGenBuffers(1,&vboCubeEdge);
    glBindVertexArray(vaoCubeEdge);
    glBindBuffer(GL_ARRAY_BUFFER, vboCubeEdge);
    glBufferData(GL_ARRAY_BUFFER,
                 edgeVerts.size()*sizeof(glm::vec3), edgeVerts.data(),
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

    if (!vaoCubeFill) glGenVertexArrays(1,&vaoCubeFill);
    if (!vboCubeFill) glGenBuffers(1,&vboCubeFill);
    glBindVertexArray(vaoCubeFill);
    glBindBuffer(GL_ARRAY_BUFFER, vboCubeFill);
    glBufferData(GL_ARRAY_BUFFER,
                 faceVerts.size()*sizeof(glm::vec3), faceVerts.data(),
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

    glBindVertexArray(0);
}

static void buildSphere(int stacks = 8, int slices = 12)
{
    std::vector<glm::vec3> v;
    std::vector<uint32_t>  idx;
    v.reserve((stacks+1)*(slices+1));

    // unit sphere (radius 0.5 for convenience)
    for (int i=0; i<=stacks; ++i) {
        float phi = (float)i * glm::pi<float>() / (float)stacks;        // 0..pi
        float y   = 0.5f * std::cos(phi);
        float r   = 0.5f * std::sin(phi);
        for (int j=0; j<=slices; ++j) {
            float th = (float)j * (2.0f * glm::pi<float>()) / (float)slices; // 0..2pi
            float x = r * std::cos(th);
            float z = r * std::sin(th);
            v.emplace_back(x,y,z);
        }
    }
    auto vid = [&](int i,int j){ return (uint32_t)(i*(slices+1) + j); };
    for (int i=0; i<stacks; ++i) {
        for (int j=0; j<slices; ++j) {
            uint32_t a = vid(i,   j);
            uint32_t b = vid(i+1, j);
            uint32_t c = vid(i+1, j+1);
            uint32_t d = vid(i,   j+1);
            idx.insert(idx.end(), { a,b,c,  a,c,d });
        }
    }
    sphereIndexCount = (GLsizei)idx.size();

    if (!vaoSphere) glGenVertexArrays(1,&vaoSphere);
    if (!vboSphere) glGenBuffers(1,&vboSphere);
    if (!eboSphere) glGenBuffers(1,&eboSphere);

    glBindVertexArray(vaoSphere);
    glBindBuffer(GL_ARRAY_BUFFER, vboSphere);
    glBufferData(GL_ARRAY_BUFFER, v.size()*sizeof(glm::vec3), v.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboSphere);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size()*sizeof(uint32_t), idx.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
}

//---------------------------------- Save Functions -----------------------------
// ------------------- Scene I/O helpers -------------------
static std::string sanitizeName(const std::string& s){
    std::string out; out.reserve(s.size());
    // Replace Windows-illegal filename chars:  \ / : * ? " < > | and control chars
    for (unsigned char c : s) {
        if (c < 32 || c == '\\' || c == '/' || c == ':' || c == '*' ||
            c == '?' || c == '"'  || c == '<' || c == '>' || c == '|')
            out.push_back('_');
        else
            out.push_back((char)c);
    }
    // Trim trailing spaces/dots (Windows doesn’t like them)
    while (!out.empty() && (out.back()==' ' || out.back()=='.')) out.pop_back();
    if (out.empty()) out = "Scene";
    return out;
}

static std::string defaultSceneName(){
    using namespace std::chrono;
    auto now   = system_clock::now();
    std::time_t tt = system_clock::to_time_t(now);
    std::tm lt {};
#ifdef _WIN32
    localtime_s(&lt, &tt);
#else
    lt = *std::localtime(&tt);
#endif
    std::ostringstream oss;
    // “Scene 2025-08-09 14-22-35”
    oss << "Scene " << std::put_time(&lt, "%Y-%m-%d %H-%M-%S");
    return sanitizeName(oss.str());
}

static bool ensureDir(const std::filesystem::path& p, std::string* err=nullptr){
    std::error_code ec;
    std::filesystem::create_directories(p, ec);
    if (ec){
        if (err) *err = "Failed to create directory: " + p.string() + " (" + ec.message() + ")";
        return false;
    }
    return true;
}

static bool writeAnchorsCSV(const std::filesystem::path& file, std::string* err){
    std::ofstream f(file, std::ios::trunc);
    if (!f){ if(err)*err="Failed to open "+file.string(); return false; }
    f << "name,x,y,z,visible,focused\n";
    for (const auto& a : gAnchors){
        f << '"' << a.name << '"' << ','
          << a.pos.x << ',' << a.pos.y << ',' << a.pos.z << ','
          << (a.visible ? 1 : 0) << ','
          << (a.focused ? 1 : 0) << '\n';
    }
    return true;
}

static bool writeDebugCubesCSV(const std::filesystem::path& file, std::string* err){
    std::ofstream f(file, std::ios::trunc);
    if (!f){ if(err)*err="Failed to open "+file.string(); return false; }
    f << "is_original,pos_x,pos_y,pos_z,rot_w,rot_x,rot_y,rot_z,scale\n";
    // original
    f << 1 << ','
      << gCube.pos.x << ',' << gCube.pos.y << ',' << gCube.pos.z << ','
      << gCube.rot.w << ',' << gCube.rot.x << ',' << gCube.rot.y << ',' << gCube.rot.z << ','
      << gCube.scale << '\n';
    // committed clones
    for (const auto& c : gExtraCubes){
        f << 0 << ','
          << c.pos.x << ',' << c.pos.y << ',' << c.pos.z << ','
          << c.rot.w << ',' << c.rot.x << ',' << c.rot.y << ',' << c.rot.z << ','
          << c.scale << '\n';
    }
    return true;
}

static bool readAnchorsCSV(const std::filesystem::path& file, std::string* err){
    std::ifstream f(file);
    if (!f){ if(err)*err="Failed to open "+file.string(); return false; }
    gAnchors.clear();
    gFocusedAnchor = -1;

    std::string line;
    // skip header
    if (!std::getline(f, line)) return true;

    auto toBool = [](std::string v)->bool{
        for (auto& c : v) c = (char)std::tolower((unsigned char)c);
        return (v=="1"||v=="true"||v=="yes"||v=="y");
    };

    while (std::getline(f, line)){
        if (line.empty()) continue;

        // naive CSV split (handles quoted name without commas inside)
        std::string name; float x=0,y=0,z=0; int vis=0, foc=0;

        // parse name (quoted or not)
        size_t i=0;
        if (i<line.size() && line[i]=='"'){
            size_t j = line.find('"', i+1);
            if (j==std::string::npos) { name = line.substr(1); }
            else { name = line.substr(i+1, j-(i+1)); i = j+1; if (i<line.size() && line[i]==',') ++i; }
        } else {
            size_t j = line.find(',', i);
            name = (j==std::string::npos) ? line.substr(i) : line.substr(i, j-i);
            i = (j==std::string::npos) ? line.size() : j+1;
        }

        auto next = [&](float& out){
            size_t j = line.find(',', i);
            std::string s = (j==std::string::npos) ? line.substr(i) : line.substr(i, j-i);
            out = std::strtof(s.c_str(), nullptr);
            i = (j==std::string::npos) ? line.size() : j+1;
        };
        auto nextI = [&](int& out){
            size_t j = line.find(',', i);
            std::string s = (j==std::string::npos) ? line.substr(i) : line.substr(i, j-i);
            out = (int)std::strtol(s.c_str(), nullptr, 10);
            i = (j==std::string::npos) ? line.size() : j+1;
        };

        next(x); next(y); next(z); nextI(vis); nextI(foc);

        Anchor a; a.name = name; a.pos = {x,y,z}; a.visible = (vis!=0); a.focused = (foc!=0);
        gAnchors.push_back(std::move(a));
    }

    // restore focused index from per-row flag
    for (int idx=0; idx<(int)gAnchors.size(); ++idx){
        if (gAnchors[idx].focused){ gFocusedAnchor = idx; break; }
    }
    return true;
}

static bool readDebugCubesCSV(const std::filesystem::path& file, std::string* err){
    std::ifstream f(file);
    if (!f){ if(err)*err="Failed to open "+file.string(); return false; }
    gExtraCubes.clear();

    std::string line;
    // skip header
    if (!std::getline(f, line)) return true;

    bool haveOriginal = false;

    while (std::getline(f, line)){
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string cell;

        auto nextF = [&](float& v){ std::getline(ss, cell, ','); v = std::strtof(cell.c_str(), nullptr); };
        auto nextI = [&](int& v){ std::getline(ss, cell, ','); v = (int)std::strtol(cell.c_str(), nullptr, 10); };

        int isOrig = 0; nextI(isOrig);
        CubeState C;
        nextF(C.pos.x); nextF(C.pos.y); nextF(C.pos.z);
        nextF(C.rot.w); nextF(C.rot.x); nextF(C.rot.y); nextF(C.rot.z);
        nextF(C.scale);

        if (isOrig){
            gCube = C; gCube.selected = false;
            haveOriginal = true;
        } else {
            gExtraCubes.push_back(C);
        }
    }

    if (!haveOriginal){
        // if file had only clones (shouldn't), keep current gCube as-is
    }
    return true;
}

// ----- Imported meshes save/load helpers -------------------------------------
static void destroyMeshGPU(ImportedMesh& m)
{
    if (m.ebo) { glDeleteBuffers(1, &m.ebo); m.ebo = 0; }
    if (m.vbo) { glDeleteBuffers(1, &m.vbo); m.vbo = 0; }
    if (m.vao) { glDeleteVertexArrays(1, &m.vao); m.vao = 0; }
    m.indexCount = 0;
}

static void destroyAllImportedMeshesGPU()
{
    for (auto& m : gMeshes) destroyMeshGPU(m);
}

// Ensure unique filename inside dstDir (adds "__1", "__2", … before extension)
static std::string copyToUnique(const std::filesystem::path& src,
                                const std::filesystem::path& dstDir,
                                std::error_code& ec)
{
    using namespace std::filesystem;
    create_directories(dstDir, ec);
    std::string base = src.filename().string();
    path candidate = dstDir / base;
    if (!exists(candidate)) {
        copy_file(src, candidate, copy_options::overwrite_existing, ec);
        return candidate.filename().string();
    }
    // add suffixes
    std::string stem = src.stem().string();
    std::string ext  = src.extension().string();
    for (int i=1; i<10000; ++i) {
        path cand = dstDir / (stem + "__" + std::to_string(i) + ext);
        if (!exists(cand)) {
            copy_file(src, cand, copy_options::overwrite_existing, ec);
            return cand.filename().string();
        }
    }
    ec = std::make_error_code(std::errc::file_exists);
    return {};
}

// Write imported_models.csv (file,name,px,py,pz,rw,rx,ry,rz)
static bool writeImportedMeshesCSV(const std::filesystem::path& otherDir,
                                   const std::filesystem::path& filesDir,
                                   std::string* err)
{
    using namespace std::filesystem;
    std::error_code ec;
    create_directories(otherDir, ec);
    create_directories(filesDir, ec);

    // Copy files + remember unique filenames used
    for (auto& m : gMeshes) {
        if (m.srcPath.empty()) continue; // safety
        ec.clear();
        std::string saved = copyToUnique(m.srcPath, filesDir, ec);
        if (ec || saved.empty()) {
            if (err) *err = "Failed copying imported mesh: " + m.srcPath.string();
            return false;
        }
        m.savedFile = saved;
    }

    std::ofstream f(otherDir / "imported_models.csv", std::ios::trunc);
    if (!f) { if (err) *err = "Cannot create imported_models.csv"; return false; }
    f << "file,name,px,py,pz,rw,rx,ry,rz\n";
    for (const auto& m : gMeshes) {
        const glm::vec3& p = m.pos;
        const glm::quat& q = m.rot; // (w,x,y,z)
        f << m.savedFile << ','
          << m.name      << ','
          << std::fixed << std::setprecision(6)
          << p.x << ',' << p.y << ',' << p.z << ','
          << q.w << ',' << q.x << ',' << q.y << ',' << q.z << '\n';
    }
    return true;
}

// Read imported_models.csv and load meshes from "3D files"
static bool readImportedMeshesCSV(const std::filesystem::path& otherDir,
                                  const std::filesystem::path& filesDir,
                                  std::string* err)
{
    using namespace std::filesystem;

    // Clean previous GPU data to avoid leaks, then reset list
    destroyAllImportedMeshesGPU();
    gMeshes.clear();
    gSelectedMesh = -1;

    const path csv = otherDir / "imported_models.csv";
    if (!exists(csv)) {
        // no imports saved — not an error
        return true;
    }

    std::ifstream f(csv);
    if (!f) { if (err) *err = "Cannot open imported_models.csv"; return false; }

    auto split = [](const std::string& s){
        std::vector<std::string> t;
        size_t p=0;
        while (true) {
            size_t c = s.find(',', p);
            if (c == std::string::npos) { t.push_back(s.substr(p)); break; }
            t.push_back(s.substr(p, c - p)); p = c + 1;
        }
        return t;
    };

    std::string ln;
    // skip header if present
    if (std::getline(f, ln)) {
        if (ln.rfind("file,", 0) != 0) { f.clear(); f.seekg(0); }
    }

    while (std::getline(f, ln)) {
        if (ln.empty()) continue;
        auto t = split(ln);
        if (t.size() < 9) continue; // file,name,px,py,pz,rw,rx,ry,rz

        std::string fileName = t[0];
        std::string name     = t[1];

        path full = filesDir / fileName;
        if (!exists(full)) {
            if (err) *err = "Missing imported mesh file: " + full.string();
            return false;
        }

        // load OBJ/STL
        CpuMesh cm; std::string e2;
        std::string ext = full.extension().string();
        std::transform(ext.begin(),ext.end(),ext.begin(),::tolower);
        bool ok = false;
        if (ext == ".obj") ok = loadOBJ_Simple(full, cm, &e2);
        else if (ext == ".stl") ok = loadSTL_Simple(full, cm, &e2);
        else {
            if (err) *err = "Unsupported imported format: " + ext;
            return false;
        }
        if (!ok) { if (err) *err = e2; return false; }

        ImportedMesh M;
        M.name      = name;
        M.srcPath   = full;          // now “source” is the saved copy
        M.savedFile = fileName;

        // parse transform
        M.pos = glm::vec3(std::stof(t[2]), std::stof(t[3]), std::stof(t[4]));
        glm::quat q; q.w = std::stof(t[5]); q.x = std::stof(t[6]); q.y = std::stof(t[7]); q.z = std::stof(t[8]);
        M.rot = glm::normalize(q);

        uploadMeshToGPU(cm, M);
        gMeshes.push_back(std::move(M));
    }
    return true;
}

// --- Ray/triangle (Möller–Trumbore) in LOCAL space ---
static bool rayTri(const glm::vec3& ro, const glm::vec3& rd,
                   const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                   float& tOut, float& uOut, float& vOut)
{
    const float EPS = 1e-7f;
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 p  = glm::cross(rd, ac);
    float det = glm::dot(ab, p);
    if (std::fabs(det) < EPS) return false;
    float inv = 1.0f / det;
    glm::vec3 s = ro - a;
    float u = glm::dot(s, p) * inv; if (u < 0.f || u > 1.f) return false;
    glm::vec3 q = glm::cross(s, ab);
    float v = glm::dot(rd, q) * inv; if (v < 0.f || u+v > 1.f) return false;
    float t = glm::dot(ac, q) * inv; if (t <= 0.f) return false;
    tOut=t; uOut=u; vOut=v; return true;
}

struct PickHit {
    enum Kind { NONE, CUBE, MESH, ANCHOR, CAMERA } kind = NONE;
    int  idx = -1;           // mesh/cube/anchor/camera index
    glm::vec3 pLocal{0};     // used for cubes/meshes
    glm::vec3 nLocal{0};     // used for cubes/meshes
    int triStart = -1;       // meshes only
    float t = 1e30f;         // world-space param along ray
};


// test a cube (in LOCAL cube space = unit [-0.5,0.5]^3, but we transform the ray into that local space)
static bool pickCubeLocal(const glm::vec3& roL, const glm::vec3& rdL, float& tHit, glm::vec3& pLocal, glm::vec3& nLocal)
{
    // Ray-box slab method to entry point
    glm::vec3 tmin = (-0.5f - roL) / rdL;
    glm::vec3 tmax = ( 0.5f - roL) / rdL;
    glm::vec3 t0 = glm::min(tmin,tmax), t1 = glm::max(tmin,tmax);
    float tEnter = std::max({t0.x,t0.y,t0.z});
    float tExit  = std::min({t1.x,t1.y,t1.z});
    if (!(tEnter <= tExit) || tExit <= 0.f) return false;
    tHit   = (tEnter>0.f? tEnter : tExit);
    pLocal = roL + tHit * rdL;

    // largest abs component indicates face normal
    glm::vec3 ap = glm::abs(pLocal);
    if (ap.x>ap.y && ap.x>ap.z) nLocal = glm::vec3((pLocal.x>0)?+1:-1,0,0);
    else if (ap.y>ap.x && ap.y>ap.z) nLocal = glm::vec3(0,(pLocal.y>0)?+1:-1,0);
    else nLocal = glm::vec3(0,0,(pLocal.z>0)?+1:-1);
    return true;
}

static bool gPickPreferUnselected = false;


static PickHit pickFacesOrPoints(const glm::vec3& roW, const glm::vec3& rdW)
{
    PickHit best;

    auto pushWorldT = [&](const glm::mat4& M, float tLocal, PickHit& tmp){
        // get world t from local t by transforming two points
        glm::vec3 p0W = glm::vec3(M * glm::vec4(0,0,0,1));
        glm::vec3 pW  = glm::vec3(M * glm::vec4(tmp.pLocal,1));

float tApprox = glm::dot(pW - roW, rdW); // rdW is unit-length
tmp.t = tApprox;

// Always take a clearly closer hit.
if (tmp.t + 1e-6f < best.t) {
    best = tmp;
    return;
}

// For ties or near-ties, make order deterministic and overlap-aware.
auto cubeCenter = [&](int cubeIdx)->glm::vec3 {
    if (cubeIdx == 0) return gCube.pos;
    int ei = cubeIdx - 1;
    if (ei >= 0 && ei < (int)gExtraCubes.size()) return gExtraCubes[ei].pos;
    return glm::vec3(0);
};
auto cubeScale = [&](int cubeIdx)->float {
    if (cubeIdx == 0) return gCube.scale;
    int ei = cubeIdx - 1;
    if (ei >= 0 && ei < (int)gExtraCubes.size()) return gExtraCubes[ei].scale;
    return 1.0f;
};

// Consider near equal t OR near-equal positions as a "tie".
const float nearT     = std::fabs(tmp.t - best.t) <= 1e-4f * std::max(1.0f, std::fabs(best.t));
bool tieOrOverlap = false;

if (tmp.kind == PickHit::CUBE && best.kind == PickHit::CUBE) {
    glm::vec3 cA = cubeCenter(tmp.idx);
    glm::vec3 cB = cubeCenter(best.idx);
    float sA = cubeScale(tmp.idx);
    float sB = cubeScale(best.idx);
    float thr = std::max(1e-4f, 0.02f * std::max(sA, sB)); // ~2% of size (robust for coincident clones)
    bool nearSamePos = glm::length(cA - cB) <= thr;

    tieOrOverlap = (nearT || nearSamePos);

    if (tieOrOverlap) {
        bool tmpSel  = std::find(gSelCubes.begin(), gSelCubes.end(), tmp.idx)  != gSelCubes.end();
        bool bestSel = std::find(gSelCubes.begin(), gSelCubes.end(), best.idx) != gSelCubes.end();

        // If Ctrl/Shift are held, we prefer picking something unselected.
        // On plain click, prefer currently selected to keep selection stable.
        if (gPickPreferUnselected) {
            if (!tmpSel && bestSel) { best = tmp; return; }
        } else {
            if ( tmpSel && !bestSel) { best = tmp; return; }
        }

        // Still tied? Prefer later (higher index) for determinism.
        if (tmpSel == bestSel && tmp.idx > best.idx) {
            best = tmp; return;
        }
    }
}

// Fall back to "closer wins" if not tied/overlapped.
if (tmp.t < best.t) best = tmp;
    };


    // --- Cubes (debug + extras) ---
    auto testCube = [&](const CubeState& C, int cubeIdx){
        glm::mat4 M = glm::translate(glm::mat4(1.f), C.pos)
                    * glm::mat4_cast(C.rot)
                    * glm::scale(glm::mat4(1.f), glm::vec3(C.scale));
        glm::mat4 invM = glm::inverse(M);
        glm::vec3 roL = glm::vec3(invM * glm::vec4(roW,1));
        glm::vec3 rdL = glm::normalize(glm::vec3(invM * glm::vec4(rdW,0)));
        float tHit; glm::vec3 pL,nL;
        if (pickCubeLocal(roL, rdL, tHit, pL, nL)) {
            PickHit tmp; tmp.kind=PickHit::CUBE; tmp.idx=cubeIdx; tmp.pLocal=pL; tmp.nLocal=nL; tmp.triStart=-1;
            pushWorldT(M, tHit, tmp);
        }
    };
    // visible debug cube?
    if (S.debugCube) {
        testCube(gCube, 0);
        for (int i=0;i<(int)gExtraCubes.size();++i) testCube(gExtraCubes[i], i+1);
    }

    // --- Imported meshes (need CPU tris) ---
    for (int mi=0; mi<(int)gMeshes.size(); ++mi) {
        const auto& Msh = gMeshes[mi];
        if (Msh.cpuVerts.empty() || Msh.cpuIndices.size()<3) continue;

        glm::mat4 M = glm::translate(glm::mat4(1.f), Msh.pos)
                    * glm::mat4_cast(Msh.rot)
                    * glm::scale(glm::mat4(1.f), glm::vec3(Msh.scale));
        glm::mat4 invM = glm::inverse(M);
        glm::vec3 roL = glm::vec3(invM * glm::vec4(roW,1));
        glm::vec3 rdL = glm::normalize(glm::vec3(invM * glm::vec4(rdW,0)));

        float bestTlocal = 1e30f;
        int bestTri = -1; glm::vec3 bestPL(0), bestNL(0);

        for (size_t k=0; k+2 < Msh.cpuIndices.size(); k+=3) {
            const glm::vec3& a = Msh.cpuVerts[Msh.cpuIndices[k+0]];
            const glm::vec3& b = Msh.cpuVerts[Msh.cpuIndices[k+1]];
            const glm::vec3& c = Msh.cpuVerts[Msh.cpuIndices[k+2]];
            float t,u,v;
            if (rayTri(roL, rdL, a,b,c, t,u,v)) {
                if (t < bestTlocal) {
                    bestTlocal = t;
                    bestTri = (int)k;
                    bestPL  = roL + t*rdL;
                    bestNL  = glm::normalize(glm::cross(b-a, c-a));
                }
            }
        }
        if (bestTri >= 0) {
            PickHit tmp; tmp.kind=PickHit::MESH; tmp.idx=mi; tmp.pLocal=bestPL; tmp.nLocal=bestNL; tmp.triStart=bestTri;
            pushWorldT(M, bestTlocal, tmp);
        }
    }

    // --- Viewport anchors (visible only) ---
    {
        float span = std::max({ S.width_m, S.length_m, S.depth_m, 1.0f });
        float R = std::clamp(0.02f * span, 0.08f, 1.5f);
        for (int i = 0; i < (int)gAnchors.size(); ++i) {
            const Anchor& a = gAnchors[i];
            if (!a.visible) continue;  // clickable only when visible
            glm::vec3 oc = roW - a.pos;
            float b = glm::dot(oc, rdW);
            float c = glm::dot(oc, oc) - R*R;
            float disc = b*b - c;
            if (disc >= 0.0f) {
                float s = std::sqrt(disc);
                float tHit = -b - s; if (tHit < 0.0f) tHit = -b + s;
                if (tHit > 0.0f && tHit < best.t) {
                    PickHit tmp; tmp.kind = PickHit::ANCHOR; tmp.idx = i; tmp.t = tHit;
                    best = tmp;
                }
            }
        }
    }

    // --- Scene cameras (visible only; not inside camera subviews) ---
    if (!gInCamSubview) {
        float span = std::max({ S.width_m, S.length_m, S.depth_m, 1.0f });
        float Rc = std::max(0.05f * span, 0.05f);  // small sphere at camera origin
        for (int i = 0; i < (int)gSceneCams.size(); ++i) {
            const SceneCam& c = gSceneCams[i];
            if (!c.visible) continue;
            glm::vec3 oc = roW - c.pos;
            float b = glm::dot(oc, rdW);
            float cc = glm::dot(oc, oc) - Rc*Rc;
            float disc = b*b - cc;
            if (disc >= 0.0f) {
                float s = std::sqrt(disc);
                float tHit = -b - s; if (tHit < 0.0f) tHit = -b + s;
                if (tHit > 0.0f && tHit < best.t) {
                    PickHit tmp; tmp.kind = PickHit::CAMERA; tmp.idx = i; tmp.t = tHit;
                    best = tmp;
                }
            }
        }
    }


    return best;
}

static bool saveSceneToFolder(const std::filesystem::path& base, std::string* err){
    using std::filesystem::path;

    // Make dirs
    const path d3      = base / "3D files";
    const path tex     = base / "Texture Files";
    const path acam    = base / "Anchors and Camera files";
    const path other   = base / "Other Scene data";

    if (!ensureDir(base, err)) return false;
    if (!ensureDir(d3,   err)) return false;
    if (!ensureDir(tex,  err)) return false;
    if (!ensureDir(acam, err)) return false;
    if (!ensureDir(other,err)) return false;

    // Write anchors/camera (camera later; for now anchors only)
    if (!writeAnchorsCSV(acam / "VP Anchor Points.csv", err)) return false;

    // --- Cameras CSV ---
    {
        const std::filesystem::path acDir = base / "Anchors and Camera files";
        std::error_code ecAC;
        std::filesystem::create_directories(acDir, ecAC);
        const std::filesystem::path camsCsv = acDir / "cameras.csv";

        std::ofstream f(camsCsv, std::ios::trunc);
        if (f) {
            f << "name,visible,focused,px,py,pz,dx,dy,dz,focal_mm,aperture_f,fov_deg,sensor_w_mm,sensor_h_mm\n";
            for (const auto& c : gSceneCams) {
                glm::vec3 fwd = glm::normalize(c.rot * glm::vec3(0,0,-1));
                f << c.name << ','
                  << (c.visible ? "1" : "0") << ','
                  << (c.focused ? "1" : "0") << ','
                  << c.pos.x << ',' << c.pos.y << ',' << c.pos.z << ','
                  << fwd.x  << ',' << fwd.y  << ',' << fwd.z  << ','
                  << c.focal_mm << ',' << c.aperture_f << ',' << c.fov_deg << ','
                  << c.sensor_w_mm << ',' << c.sensor_h_mm << '\n';
            }
        }
    }

    // Write debug cubes into "Other Scene data"
    if (!writeDebugCubesCSV(other / "debug_cubes.csv", err)) return false;

    // --- NEW: Imported meshes (copy files + write CSV) ---
    if (!writeImportedMeshesCSV(other, d3, err)) return false;


    return true;
}

static bool loadSceneFromFolder(const std::filesystem::path& base, std::string* err){
    using std::filesystem::path;
    const path acam  = base / "Anchors and Camera files";
    const path other = base / "Other Scene data";

    // Anchors
    if (!readAnchorsCSV(acam / "VP Anchor Points.csv", err)) return false;

// --- Load cameras.csv ---
{
    gSceneCams.clear();
    gSelectedCam = -1;

    const std::filesystem::path camsCsv = acam / "cameras.csv";
    if (std::filesystem::exists(camsCsv)) {
        std::ifstream f(camsCsv);
        std::string ln;
        // skip header if present
        if (std::getline(f, ln)) {
            if (ln.rfind("name,", 0) != 0) { f.clear(); f.seekg(0); }
        }
        while (std::getline(f, ln)) {
            if (ln.empty()) continue;

            std::vector<std::string> t; t.reserve(16);
            size_t p=0;
            while (true) {
                size_t c = ln.find(',', p);
                if (c == std::string::npos) { t.push_back(ln.substr(p)); break; }
                t.push_back(ln.substr(p, c-p));
                p = c+1;
            }
            if (t.size() < 9) continue; // need pos(3) + dir(3) at least

            SceneCam sc;
            sc.name    = t[0];
            sc.visible = (t[1]=="1"||t[1]=="true");
            sc.focused = (t[2]=="1"||t[2]=="true");
            sc.pos     = glm::vec3(std::stof(t[3]), std::stof(t[4]), std::stof(t[5]));
            glm::vec3 dir = glm::normalize(glm::vec3(std::stof(t[6]), std::stof(t[7]), std::stof(t[8])));

            // rebuild quaternion from forward; assume world up ~ +Y to define roll=0
            glm::vec3 up(0,1,0);
            if (std::abs(glm::dot(dir, up)) > 0.99f) up = glm::vec3(1,0,0);
            glm::vec3 r = glm::normalize(glm::cross(dir, up));
            glm::vec3 u = glm::normalize(glm::cross(r, dir));
            glm::mat3 M(r, u, -dir);
            sc.rot = glm::quat_cast(M);

            // Lens (optional)
            if (t.size() >= 12) {
                sc.focal_mm   = std::strtof(t[9].c_str(),  nullptr);
                sc.aperture_f = std::strtof(t[10].c_str(), nullptr);
                sc.fov_deg    = std::strtof(t[11].c_str(), nullptr);
            }
            // Sensors (optional; new files)
            if (t.size() >= 14) {
                sc.sensor_w_mm = std::strtof(t[12].c_str(), nullptr);
                sc.sensor_h_mm = std::strtof(t[13].c_str(), nullptr);
            } else {
                sc.sensor_w_mm = 36.0f;
                sc.sensor_h_mm = 24.0f;
            }

            // Keep focal <-> FOV coherent using current sensor height
            if (t.size() < 12) {
                // Old file: derive FOV from default focal
                float fovy = fovY_from_focal_mm(sc.focal_mm, sc.sensor_h_mm);
                sc.fov_deg = glm::degrees(fovy);
            }

            gSceneCams.push_back(std::move(sc));
        }
    }
}

    // ----- Imported meshes -----
    {
        const std::filesystem::path filesDir = base / "3D files";
        if (!readImportedMeshesCSV(other, filesDir, err)) return false;
        gSelectedMesh = -1; // nothing selected on load
    }



    // Debug cubes
    if (!readDebugCubesCSV(other / "debug_cubes.csv", err)) return false;

    // Optional: center view on content
    // frameAll();

    return true;
}

// -------------------- Windows file picker for 3D files ------------------

#ifdef _WIN32
#include <shobjidl.h>
static bool pickOpen3DFileWin32(std::filesystem::path& outPath) {
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    IFileOpenDialog* pDlg = nullptr;
    bool ok = false;
    if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_ALL, IID_PPV_ARGS(&pDlg)))) {
        // Filters
        COMDLG_FILTERSPEC types[] = {
            { L"3D Models (*.obj;*.stl;*.amf;*.step;*.stp;*.igs;*.iges;*.f3d)", L"*.obj;*.stl;*.amf;*.step;*.stp;*.igs;*.iges;*.f3d" },
            { L"OBJ (*.obj)", L"*.obj" },
            { L"STL (*.stl)", L"*.stl" },
            { L"All Files (*.*)", L"*.*" }
        };
        pDlg->SetFileTypes((UINT)(sizeof(types)/sizeof(types[0])), types);
        pDlg->SetFileTypeIndex(1);
        DWORD opts = 0; pDlg->GetOptions(&opts);
        pDlg->SetOptions(opts | FOS_FORCEFILESYSTEM | FOS_PATHMUSTEXIST | FOS_FILEMUSTEXIST);
        if (SUCCEEDED(pDlg->Show(nullptr))) {
            IShellItem* item = nullptr;
            if (SUCCEEDED(pDlg->GetResult(&item))) {
                PWSTR psz = nullptr;
                if (SUCCEEDED(item->GetDisplayName(SIGDN_FILESYSPATH, &psz))) {
                    outPath = std::filesystem::path(psz);
                    ok = true;
                    CoTaskMemFree(psz);
                }
                item->Release();
            }
        }
        pDlg->Release();
    }
    CoUninitialize();
    return ok;
}
#endif

// -----------------------------------------------------------------------
#ifdef _WIN32
// Native Windows folder picker using IFileDialog (Vista+)
static bool pickFolderWin32(const std::filesystem::path& start, std::filesystem::path& out){
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    bool ok = false;
    if (SUCCEEDED(hr)) {
        IFileDialog* pfd = nullptr;
        hr = CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pfd));
        if (SUCCEEDED(hr) && pfd) {
            DWORD opts = 0; pfd->GetOptions(&opts);
            pfd->SetOptions(opts | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);

            // Start folder (best-effort)
            IShellItem* startItem = nullptr;
            if (!start.empty() &&
                SUCCEEDED(SHCreateItemFromParsingName(start.wstring().c_str(), nullptr, IID_PPV_ARGS(&startItem)))) {
                pfd->SetFolder(startItem);
                startItem->Release();
                }

            if (SUCCEEDED(pfd->Show(nullptr))) {
                IShellItem* result = nullptr;
                if (SUCCEEDED(pfd->GetResult(&result)) && result) {
                    PWSTR pszPath = nullptr;
                    if (SUCCEEDED(result->GetDisplayName(SIGDN_FILESYSPATH, &pszPath)) && pszPath) {
                        out = std::filesystem::path(pszPath);
                        CoTaskMemFree(pszPath);
                        ok = true;
                    }
                    result->Release();
                }
            }
            pfd->Release();
        }
        CoUninitialize();
    }
    return ok;
}
#else
// Non-Windows fallback (returns false so the menu does nothing)
static bool pickFolderWin32(const std::filesystem::path&, std::filesystem::path&){
    return false;
}
#endif

// Save-dialog that lets the user TYPE a scene name and choose the parent folder.
// We suggest G.currentSavesDir as the starting folder and "Scene yyyy-mm-dd HH-MM-SS" as the default name.
// Returns the *folder path to create/use* in `out`.
static bool pickSaveSceneFolderWin32(const std::filesystem::path& startParent,
                                     const std::wstring& defaultSceneName,
                                     std::filesystem::path& out)
{
#ifdef _WIN32
    HRESULT hr;
    // Ensure COM is ready (same pattern you use elsewhere)
    bool didCoInit = false;
    hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    if (SUCCEEDED(hr)) didCoInit = true;

    IFileSaveDialog* pDlg = nullptr;
    hr = CoCreateInstance(CLSID_FileSaveDialog, nullptr, CLSCTX_INPROC_SERVER,
                          IID_PPV_ARGS(&pDlg));
    if (FAILED(hr)) { if (didCoInit) CoUninitialize(); return false; }

    // Default folder (current saves dir)
    IShellItem* pFolder = nullptr;
    std::wstring wStart = startParent.wstring();
    hr = SHCreateItemFromParsingName(wStart.c_str(), nullptr, IID_PPV_ARGS(&pFolder));
    if (SUCCEEDED(hr)) {
        pDlg->SetDefaultFolder(pFolder);   // suggest this folder
        pDlg->SetFolder(pFolder);          // force initial location to this folder
        pFolder->Release();
    }

    // Put our suggested name in the filename box (user can edit)
    pDlg->SetFileName(defaultSceneName.c_str());

    // We are not saving a file; we’ll *create a folder* with that name after the dialog.
    // But we still use the save dialog so the user gets a big Explorer window + typing box.
    DWORD opts = 0;
    if (SUCCEEDED(pDlg->GetOptions(&opts))) {
        // No special flags needed—just allow new name typing and overwrite prompt if exists.
        pDlg->SetOptions(opts | FOS_OVERWRITEPROMPT | FOS_NOVALIDATE);
    }

    hr = pDlg->Show(nullptr);
    if (hr == HRESULT_FROM_WIN32(ERROR_CANCELLED)) { pDlg->Release(); if (didCoInit) CoUninitialize(); return false; }
    if (FAILED(hr)) { pDlg->Release(); if (didCoInit) CoUninitialize(); return false; }

    IShellItem* pRes = nullptr;
    hr = pDlg->GetResult(&pRes);
    if (FAILED(hr) || !pRes) { pDlg->Release(); if (didCoInit) CoUninitialize(); return false; }

    PWSTR wPath = nullptr;
    hr = pRes->GetDisplayName(SIGDN_FILESYSPATH, &wPath);
    if (SUCCEEDED(hr) && wPath) {
        // The result is a *file path* string. We’ll treat it as the folder to create.
        out = std::filesystem::path(wPath);
        CoTaskMemFree(wPath);
    }
    pRes->Release();
    pDlg->Release();
    if (didCoInit) CoUninitialize();
    return !out.empty();
#else
    (void)startParent; (void)defaultSceneName; (void)out;
    return false;
#endif
}



// --------------------------------- ImGui panel --------------------------------
static bool ioCaptures();  // forward decl needed by settingsUI()

static void settingsUI() {
    #ifdef IMGUI_HAS_VIEWPORT
        const ImGuiViewport* mainvp = ImGui::GetMainViewport();
    #endif
        // ===== Top-left "File" bar =====
    #ifdef IMGUI_HAS_VIEWPORT
        ImGui::SetNextWindowViewport(mainvp->ID);
        ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + 10, mainvp->Pos.y + 10), ImGuiCond_Always, ImVec2(0,0));
    #else
        ImGui::SetNextWindowPos(ImVec2(10,10), ImGuiCond_Always, ImVec2(0,0));
    #endif
        ImGui::Begin("FilesBar", nullptr,
                     ImGuiWindowFlags_NoTitleBar |
                     ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize);

    if (ImGui::Button("File")) ImGui::OpenPopup("files_menu");
    if (ImGui::BeginPopup("files_menu")) {
        if (ImGui::MenuItem("Save Scene")) {
            // Windows "Save" flow (choose/confirm scene folder)
            std::wstring wDefault = std::wstring(L"Scene ") + std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>>().from_bytes(defaultSceneName());
            std::filesystem::path chosen;
            if (pickSaveSceneFolderWin32(cfg::G.currentSavesDir, wDefault, chosen)) {
                std::string err;
                if (!saveSceneToFolder(chosen, &err)) {
                    std::cerr << "Save failed: " << err << "\n";
                }
            }
        }
        if (ImGui::MenuItem("Load Scene")) {
            std::filesystem::path folder;
            if (pickFolderWin32(cfg::G.currentSavesDir, folder)) {
                std::string err;
                if (!loadSceneFromFolder(folder, &err)) {
                    std::cerr << "Load failed: " << err << "\n";
                }
            }
        }

        // NEW: Import Object…
        if (ImGui::MenuItem("Import Object...")) {
            std::filesystem::path modelPath;
            // NOTE: ensure you only have ONE definition of pickOpen3DFileWin32 in the file.
            if (pickOpen3DFileWin32(modelPath)) {
                std::string err;
                int idx = importModelFromFile(modelPath, &err);
                if (idx < 0) {
                    std::cerr << "Import failed: " << err << "\n";
                }
            }
        }

        ImGui::EndPopup();
    }



    // Capture window bottom so we can push the nav cube down
    {
        ImVec2 pos = ImGui::GetWindowPos();
        ImVec2 siz = ImGui::GetWindowSize();
        gTopLeftBarBottomY = pos.y + siz.y;
    }
    ImGui::End();

    // --- Save Scene modal ---
    if (gOpenSaveModal) { ImGui::OpenPopup("Save Scene"); gOpenSaveModal = false; }

    // ===== Settings window (top-right) =====
    #ifdef IMGUI_HAS_VIEWPORT
        ImGui::SetNextWindowViewport(mainvp->ID);
        ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x - 10, mainvp->Pos.y + 10),
                                ImGuiCond_Always, ImVec2(1,0));
    #else
        ImGui::SetNextWindowPos(ImVec2(FBW-10,10), ImGuiCond_Always, ImVec2(1,0));
    #endif
        ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);


    bool usingDefaultSaves = (cfg::G.currentSavesDir == cfg::G.defaultSavesDir);

    if (ImGui::Checkbox("Dark mode", &S.dark)) {
        // Persist immediately
        cfg::saveConfig(usingDefaultSaves);
    }
    ImGui::SameLine();
    ImGui::Checkbox("Debug cube", &S.debugCube);
    ImGui::SameLine();
    ImGui::Checkbox("Show axes (X/Y/Z)", &S.showAxes);

    if (ImGui::Checkbox("Wireframe", &S.wire)) {
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
        // Persist immediately
        cfg::saveConfig(usingDefaultSaves);
    }


    if(S.wire && ImGui::SliderInt("Res 2^n", &S.res_log2, 4, 11)) buildSurface();

    float op = S.opacity * 100.f;
    if(ImGui::SliderFloat("Opacity %", &op, 0.0f, 100.0f, "%.1f")) S.opacity = op/100.f;

    if(ImGui::InputFloat("Width (m)", &S.width_m) | ImGui::InputFloat("Length (m)", &S.length_m)) buildSurface();
    ImGui::InputFloat("Depth (m)", &S.depth_m);
    ImGui::InputFloat("Viscosity", &S.visc);
    ImGui::Checkbox("Linear superposition", &S.linear);

    if(ImGui::InputInt("Wave directions", &S.nDir)){ S.nDir = std::clamp(S.nDir, 1, 32); S.waves.resize(S.nDir); }

    ImGui::BeginChild("wavechild", ImVec2(0,160), true, ImGuiWindowFlags_HorizontalScrollbar);
    for(int i=0;i<S.nDir;++i){
        ImGui::PushID(i);
        ImGui::Text("Dir %d", i); ImGui::SameLine();
        ImGui::InputFloat("Angle",  &S.waves[i].angle_deg);
        ImGui::InputFloat("Height", &S.waves[i].height_m);
        ImGui::InputFloat("Freq",   &S.waves[i].freq_hz);
        ImGui::InputFloat("λ",      &S.waves[i].lambda_m);
        ImGui::InputFloat("Decay",  &S.waves[i].decay_s);
        ImGui::Separator();
        ImGui::PopID();
    }
    ImGui::EndChild();

    ImGui::SeparatorText("Navigation cube");
    const char* cubePosItems[] = { "Top-Left (default)", "Right Side" };
    ImGui::Combo("Position", &S.cubePos, cubePosItems, IM_ARRAYSIZE(cubePosItems));
    ImGui::SeparatorText("Save Location");

    // Show current path (trimmed if very long)
    {
        std::string cur = cfg::G.currentSavesDir.string();
        if (cur.size() > 90) {
            cur = "..." + cur.substr(cur.size()-90);
        }
        ImGui::TextWrapped("Current: %s", cur.c_str());
    }

    // Open Saves Directory
    if (ImGui::Button("Open Saves Directory")) {
        cfg::openDirectoryInExplorer(cfg::G.currentSavesDir);
    }
    ImGui::SameLine();

    // Change Save Directory (folder picker)
    if (ImGui::Button("Change Save Directory")) {
        auto picked = cfg::chooseDirectoryDialog(cfg::G.currentSavesDir);
        if (!picked.empty()) {
            cfg::G.currentSavesDir = picked;
            cfg::ensureDirs();
            cfg::saveConfig(false); // use_default=false
        }
    }
    ImGui::SameLine();

    // Restore Default
    if (ImGui::Button("Restore Default")) {
        cfg::G.currentSavesDir = cfg::G.defaultSavesDir;
        cfg::ensureDirs();
        cfg::saveConfig(true);  // use_default=true
    }

    // capture Settings window rect
    gSettingsCollapsed = ImGui::IsWindowCollapsed();
    ImVec2 settingsPos  = ImGui::GetWindowPos();
    ImVec2 settingsSize = ImGui::GetWindowSize();
    gSettingsBottomY = settingsPos.y + settingsSize.y;

    ImGui::End(); // ===== end Settings =====

    gRightStackBottomY = gSettingsBottomY;  // << baseline for the right-side stack

    // ===== Viewport Anchor Points window (just below Settings) =====
static int  editingIdx = -1;
static int  nextId = 1;
static char editBuf[128] = {0};
static bool focusNameOnce = false; // focus the text field next frame
static bool anchorInit = true;     // force-collapsed on first open

// Seed a default "Origin" anchor once (hidden by default, not focused)
if (anchorInit && gAnchors.empty()) {
    gAnchors.push_back(Anchor{ "Origin", /*visible=*/false, /*focused=*/false, glm::vec3(0,0,0) });
    // nextId remains 1 so user-created anchors start at "Anchor point 1"
}

// place under Settings, right-aligned
#ifdef IMGUI_HAS_VIEWPORT
    ImGui::SetNextWindowViewport(mainvp->ID);
    ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x - 10, gSettingsBottomY + 6),
                            ImGuiCond_Always, ImVec2(1,0));
#else
    ImGui::SetNextWindowPos(ImVec2(FBW-10, gSettingsBottomY + 6), ImGuiCond_Always, ImVec2(1,0));
#endif
    if (anchorInit) {
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Always);  // default collapsed
    ImGui::SetNextWindowSize(ImVec2(240, 0), ImGuiCond_Always);
} else {
    ImGui::SetNextWindowSize(ImVec2(240, 0), ImGuiCond_FirstUseEver);
}

ImGui::Begin("Viewport Anchor Points", nullptr,
             ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);
anchorInit = false;

gAnchorCollapsed = ImGui::IsWindowCollapsed();
if (gAnchorCollapsed) {
    // keep header narrow so collapsed width is small
    ImGui::SetWindowSize(ImVec2(240, 0));
}

// tiny helper: draw an eye icon (visible or slashed) inside an InvisibleButton
auto EyeButton = [&](const char* id, bool visible)->bool {
    ImGui::PushID(id);
    const ImVec2 sz(22, 20);
    bool pressed = ImGui::InvisibleButton("##eye", sz);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetItemRectMin();
    ImVec2 p1 = ImGui::GetItemRectMax();
    ImVec2 c  = ImVec2((p0.x+p1.x)*0.5f, (p0.y+p1.y)*0.5f);
    float w = p1.x - p0.x, h = p1.y - p0.y;
    float rx = w * 0.40f, ry = h * 0.32f;
    ImU32 colLine = ImGui::GetColorU32(ImVec4(0.85f,0.85f,0.85f, ImGui::IsItemHovered()?1.0f:0.75f));
    // ellipse outline (eye)
    const int seg = 32;
    ImVec2 pts[seg];
    for (int i=0;i<seg;++i){
        float a = (float)i * (2.0f * glm::pi<float>() / seg);
        pts[i] = ImVec2(c.x + cosf(a)*rx, c.y + sinf(a)*ry);
    }
    dl->AddPolyline(pts, seg, colLine, ImDrawFlags_Closed, 1.6f);
    // pupil
    dl->AddCircleFilled(c, std::min(rx,ry)*0.28f, ImGui::GetColorU32(ImVec4(0.2f,0.2f,0.2f,1.0f)));
    if (!visible){
        // slash
        dl->AddLine(ImVec2(p0.x+2, p1.y-2), ImVec2(p1.x-2, p0.y+2), colLine, 2.0f);
    }
    ImGui::PopID();
    return pressed;
};

// tiny helper: circled X delete button
auto CircleXButton = [&](const char* id)->bool {
    ImGui::PushID(id);
    const ImVec2 sz(20, 20);
    bool pressed = ImGui::InvisibleButton("##del", sz);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetItemRectMin();
    ImVec2 p1 = ImGui::GetItemRectMax();
    ImVec2 c  = ImVec2((p0.x+p1.x)*0.5f, (p0.y+p1.y)*0.5f);
    float r = 0.45f * std::min(p1.x-p0.x, p1.y-p0.y);
    ImU32 col = ImGui::GetColorU32(ImVec4(0.85f,0.85f,0.85f, ImGui::IsItemHovered()?1.0f:0.8f));
    dl->AddCircle(c, r, col, 32, 1.6f);
    float s = r*0.65f;
    dl->AddLine(ImVec2(c.x-s,c.y-s), ImVec2(c.x+s,c.y+s), col, 1.6f);
    dl->AddLine(ImVec2(c.x-s,c.y+s), ImVec2(c.x+s,c.y-s), col, 1.6f);
    ImGui::PopID();
    return pressed;
};

if (!gAnchorCollapsed) {
    ImGui::BeginChild("anchor_list", ImVec2(320, 180), true, ImGuiWindowFlags_HorizontalScrollbar);

    // Header row
    ImGui::TextUnformatted("Name"); ImGui::SameLine(200);
    ImGui::TextUnformatted("Vis");  ImGui::SameLine(245);
    ImGui::TextUnformatted("Focus");ImGui::SameLine(300);
    ImGui::TextUnformatted("Del");
    ImGui::Separator();

    for (int i = 0; i < (int)gAnchors.size(); ++i) {
        ImGui::PushID(i);

        // Name cell (double-click to edit with proper keyboard focus)
        ImGui::SetCursorPosX(4);
        if (editingIdx == i) {
            ImGui::SetNextItemWidth(190);
            if (focusNameOnce) { ImGui::SetKeyboardFocusHere(); focusNameOnce = false; }

            bool pressEnter = ImGui::InputText("##name", editBuf, IM_ARRAYSIZE(editBuf),
                                ImGuiInputTextFlags_AutoSelectAll |
                                ImGuiInputTextFlags_EnterReturnsTrue);

            // Commit when Enter is pressed or when the item is deactivated after edit
            if (pressEnter || ImGui::IsItemDeactivatedAfterEdit()) {
                if (editBuf[0] != '\0') gAnchors[i].name = editBuf;
                editingIdx = -1;
            }
            // Lost focus without an edit (clicked away) — still apply whatever is typed
            else if (ImGui::IsItemDeactivated()) {
                if (editBuf[0] != '\0') gAnchors[i].name = editBuf;
                editingIdx = -1;
            }
        } else {
            // inside "Viewport Anchor Points" list row
            ImGui::SetNextItemWidth(190);

            // Show selected style if this index is in the multiselect list
            bool inMulti = (std::find(gSelAnchors.begin(), gSelAnchors.end(), i) != gSelAnchors.end());
            if (ImGui::Selectable(gAnchors[i].name.c_str(), inMulti,
                                  ImGuiSelectableFlags_AllowDoubleClick, ImVec2(190, 0))) {
                ImGuiIO& io = ImGui::GetIO();
                bool shift = io.KeyShift ||
                             glfwGetKey(Win, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                             glfwGetKey(Win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
                bool ctrl  = io.KeyCtrl ||
                             glfwGetKey(Win, GLFW_KEY_LEFT_CONTROL)  == GLFW_PRESS ||
                             glfwGetKey(Win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

                auto inList = [&](int v){ return std::find(gSelAnchors.begin(), gSelAnchors.end(), v) != gSelAnchors.end(); };
                auto add    = [&](int v){ if (!inList(v)) gSelAnchors.push_back(v); };
                auto remove = [&](int v){
                    auto it = std::find(gSelAnchors.begin(), gSelAnchors.end(), v);
                    if (it != gSelAnchors.end()) gSelAnchors.erase(it);
                };

                if (ctrl) {
                    if (inList(i)) {
                        bool wasPrimary = (gSelectedAnchor == i);
                        remove(i);
                        if (wasPrimary)
                            gSelectedAnchor = gSelAnchors.empty() ? -1 : gSelAnchors.front();
                    } else {
                        add(i);
                        if (gSelectedAnchor < 0) gSelectedAnchor = i;
                    }
                } else if (shift) {
                    add(i);
                    if (gSelectedAnchor < 0) gSelectedAnchor = i;
                } else {
                    // Single select
                    gSelAnchors.clear();
                    gSelAnchors.push_back(i);
                    gSelectedAnchor = i;

                    // Clear unrelated types on plain click
                    gSelectedCam  = -1; gSelCams.clear();
                    gSelectedMesh = -1; gSelectedMeshes.clear();
                    gSelectedCube = -1; gCube.selected = false;
                }

                // Ensure Move targets anchors
                gMove.obj = MoveObj::ViewportAnchors;

                if (gMove.active) refreshMoveSnapshotForSelection();
                                  }

            if (ImGui::IsItemHovered() &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                std::snprintf(editBuf, sizeof(editBuf), "%s", gAnchors[i].name.c_str());
                editingIdx = i;
                focusNameOnce = true; // focus the text field next frame
                }

        }

        // Eye (visible toggle)
        ImGui::SameLine(200);
        if (EyeButton("eye", gAnchors[i].visible)) {
            gAnchors[i].visible = !gAnchors[i].visible;
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip(gAnchors[i].visible ? "Hide anchor" : "Show anchor");

        // Focus button (toggle focus / unfocus)
        ImGui::SameLine(245);
        bool isFocused = (gFocusedAnchor == i);
        if (ImGui::SmallButton(isFocused ? "Focused" : "Focus")) {
            if (isFocused) {
                gFocusedAnchor = -1;
                for (auto& a : gAnchors) a.focused = false;
            } else {
                gFocusedAnchor = i;
                for (int j = 0; j < (int)gAnchors.size(); ++j) gAnchors[j].focused = (j == i);
                // Make the focused anchor the current orbit center
                Cam.pan = gAnchors[i].pos;
            }
        }

        // Delete
        ImGui::SameLine(300);
        if (CircleXButton("del")) {
            // 1) Update focus/edit state
            if (gFocusedAnchor == i) gFocusedAnchor = -1;
            if (editingIdx == i)     editingIdx     = -1;

            // 2) Erase the anchor
            gAnchors.erase(gAnchors.begin() + i);

            // 3) Repair multi-select & primary selection
            //    - remove deleted index
            gSelAnchors.erase(std::remove(gSelAnchors.begin(), gSelAnchors.end(), i), gSelAnchors.end());
            //    - shift all indices after 'i' down by 1
            for (int& idx : gSelAnchors) if (idx > i) --idx;

            if      (gSelectedAnchor == i) gSelectedAnchor = -1;
            else if (gSelectedAnchor  > i) --gSelectedAnchor;

            // 4) Clamp (safety)
            gSelAnchors.erase(std::remove_if(gSelAnchors.begin(), gSelAnchors.end(),
                                             [](int k){ return k < 0 || k >= (int)gAnchors.size(); }),
                              gSelAnchors.end());
            if (gSelectedAnchor >= (int)gAnchors.size()) gSelectedAnchor = -1;

            // 5) Optional: keep Move mode coherent
            if (gMove.obj == MoveObj::ViewportAnchors && gSelAnchors.empty() && gSelectedAnchor < 0)
                gMove.obj = MoveObj::Bodies;

            ImGui::PopID();
            break;
        }


        ImGui::PopID();
    }
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        !ImGui::IsAnyItemHovered())
    {
        gSelAnchors.clear();
        gSelectedAnchor = -1;
        if (gMove.obj == MoveObj::ViewportAnchors)
            gMove.obj = MoveObj::Bodies;
        if (gMove.active) refreshMoveSnapshotForSelection();
    }
    ImGui::EndChild();

    // Add button
    ImGui::SetCursorPosX(4);
    if (ImGui::Button("+", ImVec2(28, 0))) {
        Anchor a;
        a.name = "Anchor point " + std::to_string(nextId++);
        a.visible = true; a.focused = false;
        a.pos = glm::vec3(0,0,0); // default position at origin
        gAnchors.push_back(std::move(a));
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add anchor point");
}

// Publish the bottom of the right-side stack (Settings + Anchor header/body)
{
    ImVec2 aPos  = ImGui::GetWindowPos();
    ImVec2 aSize = ImGui::GetWindowSize();
    const float anchorBottom = aPos.y + aSize.y;
    gRightStackBottomY = std::max(gRightStackBottomY, anchorBottom);
}

ImGui::End(); // ===== end Anchor Points window =====

    // ===== Cameras window (under Anchor Points) =====
static bool camsInit = true;
static int  camNextId = 1;
static int  camEditingIdx = -1;
static char camEditBuf[128] = {0};
static bool camFocusNameOnce = false;
static int   camSettingsIdx   = -1;
//static float tmpFocal = 50.0f, tmpAperture = 2.8f, tmpFov = 45.0f;


#ifdef IMGUI_HAS_VIEWPORT
    ImGui::SetNextWindowViewport(mainvp->ID);
    ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x - 10, gRightStackBottomY + 6),
                            ImGuiCond_Always, ImVec2(1,0));
#else
    ImGui::SetNextWindowPos(ImVec2(FBW-10, gRightStackBottomY + 6), ImGuiCond_Always, ImVec2(1,0));
#endif
if (camsInit) {
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_Always);  // wider
} else {
    ImGui::SetNextWindowSize(ImVec2(320, 0), ImGuiCond_FirstUseEver);
}
ImGui::Begin("Cameras", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);
camsInit = false;


ImGui::BeginChild("cams_list", ImVec2(420, 200), true, 0);       // wider child, no horiz scroll
bool openCamSettingsThisFrame = false;   // NEW: defer popup open until after EndChild()


// Header row (shifted stops)
ImGui::TextUnformatted("Name");     ImGui::SameLine(200);
ImGui::TextUnformatted("Vis");      ImGui::SameLine(250);
ImGui::TextUnformatted("Focus");    ImGui::SameLine(300);
ImGui::TextUnformatted("Settings"); ImGui::SameLine(380);
ImGui::TextUnformatted("Del");
ImGui::Separator();

for (int i = 0; i < (int)gSceneCams.size(); ++i) {
    ImGui::PushID(i);

    // Name (double-click to edit; single click selects for Move)
    ImGui::SetCursorPosX(4);
    if (camEditingIdx == i) {
        ImGui::SetNextItemWidth(190);
        if (camFocusNameOnce) { ImGui::SetKeyboardFocusHere(); camFocusNameOnce = false; }
        bool pressEnter = ImGui::InputText("##name", camEditBuf, IM_ARRAYSIZE(camEditBuf),
            ImGuiInputTextFlags_AutoSelectAll | ImGuiInputTextFlags_EnterReturnsTrue);
        if (pressEnter || ImGui::IsItemDeactivatedAfterEdit() || ImGui::IsItemDeactivated()) {
            if (camEditBuf[0] != '\0') gSceneCams[i].name = camEditBuf;
            camEditingIdx = -1;
        }
    } else {
        ImGui::SetNextItemWidth(190);

        bool inMulti = (std::find(gSelCams.begin(), gSelCams.end(), i) != gSelCams.end());
        if (ImGui::Selectable(gSceneCams[i].name.c_str(), inMulti, 0, ImVec2(190,0))) {
            ImGuiIO& io = ImGui::GetIO();
            bool shift = io.KeyShift ||
                         glfwGetKey(Win, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
                         glfwGetKey(Win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            bool ctrl  = io.KeyCtrl ||
                         glfwGetKey(Win, GLFW_KEY_LEFT_CONTROL)  == GLFW_PRESS ||
                         glfwGetKey(Win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;

            auto inList = [&](int v){ return std::find(gSelCams.begin(), gSelCams.end(), v) != gSelCams.end(); };
            auto add    = [&](int v){ if (!inList(v)) gSelCams.push_back(v); };
            auto remove = [&](int v){
                auto it = std::find(gSelCams.begin(), gSelCams.end(), v);
                if (it != gSelCams.end()) gSelCams.erase(it);
            };

            if (ctrl) {
                if (inList(i)) {
                    bool wasPrimary = (gSelectedCam == i);
                    remove(i);
                    if (wasPrimary)
                        gSelectedCam = gSelCams.empty() ? -1 : gSelCams.front();
                } else {
                    add(i);
                    if (gSelectedCam < 0) gSelectedCam = i;
                }
            } else if (shift) {
                add(i);
                if (gSelectedCam < 0) gSelectedCam = i;
            } else {
                // Single select
                gSelCams.clear();
                gSelCams.push_back(i);
                gSelectedCam = i;

                // Clear unrelated types on plain click
                gSelectedAnchor = -1; gSelAnchors.clear();
                gSelectedMesh   = -1; gSelectedMeshes.clear();
                gCube.selected  = false; gSelectedCube = -1;
            }

            // Ensure Move targets cameras
            gMove.obj = MoveObj::Cameras;

            if (gMove.active) refreshMoveSnapshotForSelection();



            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                std::snprintf(camEditBuf, sizeof(camEditBuf), "%s", gSceneCams[i].name.c_str());
                camEditingIdx = i; camFocusNameOnce = true;
            }
        }
    }

    // Vis: eye icon
    // Use the already-defined EyeButton here
    ImGui::SameLine(200);
    if (EyeButton("vis", gSceneCams[i].visible)) {
        gSceneCams[i].visible = !gSceneCams[i].visible;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(gSceneCams[i].visible ? "Hide camera" : "Show camera");
    }

    // Focus (multiple allowed)
    ImGui::SameLine(250);
    bool f = gSceneCams[i].focused;
    if (ImGui::Checkbox("##focus", &f)) gSceneCams[i].focused = f;

    // Settings popup trigger
    ImGui::SameLine(300);
    if (ImGui::SmallButton("Settings")) {
        camSettingsIdx = i;
        openCamSettingsThisFrame = true;   // NEW: mark to open after we pop row ID
    }

    // Delete
    ImGui::SameLine(380);
    if (ImGui::SmallButton("X")) {
        // 1) Erase the camera
        gSceneCams.erase(gSceneCams.begin() + i);

        // 2) Repair multi-select & primary selection
        gSelCams.erase(std::remove(gSelCams.begin(), gSelCams.end(), i), gSelCams.end());
        for (int& idx : gSelCams) if (idx > i) --idx;

        if      (gSelectedCam == i) gSelectedCam = -1;
        else if (gSelectedCam  > i) --gSelectedCam;

        // 3) Clamp (safety)
        gSelCams.erase(std::remove_if(gSelCams.begin(), gSelCams.end(),
                                      [](int k){ return k < 0 || k >= (int)gSceneCams.size(); }),
                       gSelCams.end());
        if (gSelectedCam >= (int)gSceneCams.size()) gSelectedCam = -1;

        // 4) Optional: keep Move mode coherent
        if (gMove.obj == MoveObj::Cameras && gSelCams.empty() && gSelectedCam < 0)
            gMove.obj = MoveObj::Bodies;

        ImGui::PopID();
        break;
    }


    ImGui::PopID();
}
    // If user clicks blank space in the camera list (not on an item), deselect ALL cameras.
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        !ImGui::IsAnyItemHovered())
    {
        gSelCams.clear();
        gSelectedCam = -1;
        if (gMove.obj == MoveObj::Cameras)
            gMove.obj = MoveObj::Bodies;
        if (gMove.active) refreshMoveSnapshotForSelection();
    }
    ImGui::EndChild();


if (openCamSettingsThisFrame) {
        ImGui::OpenPopup("Camera Settings");   // now in same ID context as BeginPopupModal
}


// Add (+) button
ImGui::SetCursorPosX(4);
if (ImGui::Button("+", ImVec2(28, 0))) {
    SceneCam c;
    c.name = "Camera " + std::to_string(camNextId++);
    c.pos  = glm::vec3(0,0,0);
    c.rot  = glm::quat(1,0,0,0);
    gSceneCams.push_back(std::move(c));
}
if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add camera");

ImGui::SameLine();
    bool canOpen = (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size());
    if (!canOpen) ImGui::BeginDisabled();
    if (ImGui::Button("Open View")) {
        CamViewWin cv;
        cv.camIndex = gSelectedCam;
        cv.title = "Camera View: " + gSceneCams[gSelectedCam].name;
        cv.justCreated = true;                 // first frame placement
        cv.wantCenterDockThisFrame = true;     // center + size on first open
        gCamViews.push_back(std::move(cv));
    }
if (!canOpen) ImGui::EndDisabled();


// --- Camera Settings modal (live, synced with Open View) ---
if (ImGui::BeginPopupModal("Camera Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    if (camSettingsIdx >= 0 && camSettingsIdx < (int)gSceneCams.size()) {
        SceneCam& cam = gSceneCams[camSettingsIdx];

        ImGui::TextUnformatted("Lens");
        ImGui::Separator();

        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("Focal (mm)", &cam.focal_mm, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.focal_mm = std::max(0.01f, cam.focal_mm);
            float fovy = fovY_from_focal_mm(cam.focal_mm, cam.sensor_h_mm);
            cam.fov_deg = glm::degrees(fovy);
        }

        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("f-number", &cam.aperture_f, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.aperture_f = std::max(0.7f, cam.aperture_f);
        }

        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("FOV (deg)", &cam.fov_deg, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.fov_deg = std::clamp(cam.fov_deg, 1.0f, 170.0f);
            float fovy = glm::radians(cam.fov_deg);
            cam.focal_mm = focal_mm_from_fovY(fovy, cam.sensor_h_mm);
        }

        ImGui::Separator();
        ImGui::TextUnformatted("Sensor");
        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("Width (mm)", &cam.sensor_w_mm, 0, 0, "%.2f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.sensor_w_mm = std::max(1.0f, cam.sensor_w_mm);
        }

        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("Height (mm)", &cam.sensor_h_mm, 0, 0, "%.2f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.sensor_h_mm = std::max(1.0f, cam.sensor_h_mm);
            float fovy = fovY_from_focal_mm(cam.focal_mm, cam.sensor_h_mm);
            cam.fov_deg = glm::degrees(fovy);
        }

        ImGui::Separator();
    }

    if (ImGui::Button("Close", ImVec2(120,0))) {
        camSettingsIdx = -1;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}


// ---- Camera View Windows ----------------------------------------------------
for (int i = 0; i < (int)gCamViews.size(); ) {
    CamViewWin& cv = gCamViews[i];

    // Drop closed or invalid views
    if (!cv.open || cv.camIndex < 0 || cv.camIndex >= (int)gSceneCams.size()) {
        destroyCamRT(cv);
        gCamViews.erase(gCamViews.begin() + i);
        continue;
    }


    // Handle pop-out / return position on the NEXT Begin
    if (cv.wantPopMoveThisFrame) {
        const ImGuiViewport* mainvp = ImGui::GetMainViewport();
        if (cv.poppedOut) {
            // Pop-out: place outside main viewport so it becomes a platform window
            ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x + 40.0f,
                                           mainvp->Pos.y + 40.0f), ImGuiCond_Always);
        } else {
#ifdef IMGUI_HAS_VIEWPORT
            ImGui::SetNextWindowViewport(mainvp->ID);
#endif
            // If we’re just created or explicitly requested, center near top and force size
            if (cv.justCreated || cv.wantCenterDockThisFrame) {
                ImVec2 sz  = kCamDockedSize;
                ImVec2 pos = ImVec2(
                    mainvp->Pos.x + (mainvp->Size.x - sz.x) * 0.5f, // center X
                    mainvp->Pos.y + kCamDockedTopMargin             // near top
                );
                ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
                ImGui::SetNextWindowSize(sz, ImGuiCond_Always);
                cv.justCreated = false;
                cv.wantCenterDockThisFrame = false;
            } else {
                // fallback: keep previous behavior if neither flag set
                ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + 100.0f,
                                               mainvp->Pos.y + 100.0f), ImGuiCond_Always);
            }
        }
        cv.wantPopMoveThisFrame = false;
    }

#ifdef IMGUI_HAS_VIEWPORT
    // Safety: If the window is embedded (not popped out) and we’re not moving this frame,
    // but it was restored with a strange size/pos, normalize once on first frame.
    if (!cv.poppedOut && (cv.justCreated || cv.wantCenterDockThisFrame)) {
        const ImGuiViewport* mainvp = ImGui::GetMainViewport();
        ImVec2 sz  = kCamDockedSize;
        ImVec2 pos = ImVec2(
            mainvp->Pos.x + (mainvp->Size.x - sz.x) * 0.5f,
            mainvp->Pos.y + kCamDockedTopMargin
        );
        ImGui::SetNextWindowViewport(mainvp->ID);
        ImGui::SetNextWindowPos(pos, ImGuiCond_Always);
        ImGui::SetNextWindowSize(sz, ImGuiCond_Always);
        cv.justCreated = false;
        cv.wantCenterDockThisFrame = false;
    }
#endif

#ifdef IMGUI_HAS_VIEWPORT
    {
        ImGuiWindowClass wc{};
        if (cv.poppedOut) {
            // Separate OS window AND force normal OS decorations/taskbar entry
            wc.ViewportFlagsOverrideSet   = ImGuiViewportFlags_NoAutoMerge;
            wc.ViewportFlagsOverrideClear = ImGuiViewportFlags_NoDecoration | ImGuiViewportFlags_NoTaskBarIcon;
        } else {
            // Inside main viewport; also ensure we don't accidentally remove OS chrome
            wc.ViewportFlagsOverrideClear = ImGuiViewportFlags_NoAutoMerge |
                                            ImGuiViewportFlags_NoDecoration |
                                            ImGuiViewportFlags_NoTaskBarIcon;
        }
        ImGui::SetNextWindowClass(&wc);
    }
#endif

    // Transparent when embedded; normal window when popped out
    ImGuiWindowFlags vf = cv.poppedOut ? 0 : ImGuiWindowFlags_NoBackground;
    if (ImGui::Begin((cv.title + "##cv" + std::to_string(i)).c_str(), &cv.open, vf)) {

#ifdef IMGUI_HAS_VIEWPORT
        if (!cv.poppedOut) {
            const ImGuiViewport* mainvp = ImGui::GetMainViewport();
            ImVec2 pos  = ImGui::GetWindowPos();
            ImVec2 size = ImGui::GetWindowSize();
            const float pad = 2.0f; // keep a tiny border visible

            ImVec2 min = ImVec2(mainvp->Pos.x + pad,                       mainvp->Pos.y + pad);
            ImVec2 max = ImVec2(mainvp->Pos.x + mainvp->Size.x - size.x - pad,
                                mainvp->Pos.y + mainvp->Size.y - size.y - pad);

            ImVec2 clamped(
                (pos.x < min.x) ? min.x : (pos.x > max.x ? max.x : pos.x),
                (pos.y < min.y) ? min.y : (pos.y > max.y ? max.y : pos.y)
            );
            if (clamped.x != pos.x || clamped.y != pos.y)
                ImGui::SetWindowPos(clamped, ImGuiCond_Always);
        }
#endif


        // Title bar buttons
        if (ImGui::Button(cv.poppedOut ? "Return to Sim" : "Pop-Out")) {
            bool wasPopped = cv.poppedOut;
            cv.poppedOut = !cv.poppedOut;
            cv.wantPopMoveThisFrame = true;
            if (wasPopped && !cv.poppedOut) {
                cv.wantCenterDockThisFrame = true; // center+size when re-docking
            }
        }


    ImGui::SameLine();
    ImGui::TextDisabled("F: %.1fmm  f/%.1f  FOV: %.1f°",
        gSceneCams[cv.camIndex].focal_mm,
        gSceneCams[cv.camIndex].aperture_f,
        gSceneCams[cv.camIndex].fov_deg);

    // ---------- Lens controls (commit on Enter or focus loss) ----------
    SceneCam& cam = gSceneCams[cv.camIndex];
    ImGui::Separator();

    // Row 1: focal, f-number, FOV
    ImGui::BeginGroup();
    {
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Focal (mm)", &cam.focal_mm, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.focal_mm = std::max(0.01f, cam.focal_mm);
            float fovy = fovY_from_focal_mm(cam.focal_mm, cam.sensor_h_mm);
            cam.fov_deg = glm::degrees(fovy);
        }

        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("f-number", &cam.aperture_f, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.aperture_f = std::max(0.7f, cam.aperture_f);
        }

        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("FOV (deg)", &cam.fov_deg, 0, 0, "%.1f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.fov_deg = std::clamp(cam.fov_deg, 1.0f, 170.0f);
            float fovy = glm::radians(cam.fov_deg);
            cam.focal_mm = focal_mm_from_fovY(fovy, cam.sensor_h_mm);
        }

    }
    ImGui::EndGroup();

    // Row 2: sensor size (editing height directly affects vertical FOV)
    ImGui::BeginGroup();
    {
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Sensor W (mm)", &cam.sensor_w_mm, 0, 0, "%.2f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.sensor_w_mm = std::max(1.0f, cam.sensor_w_mm);
            // (no projection change; kept for horizontal FOV if you use it later)
        }

        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Sensor H (mm)", &cam.sensor_h_mm, 0, 0, "%.2f");
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            cam.sensor_h_mm = std::max(1.0f, cam.sensor_h_mm);
            // re-derive vertical FOV from focal with the new sensor height
            float fovy = fovY_from_focal_mm(cam.focal_mm, cam.sensor_h_mm);
            cam.fov_deg = glm::degrees(fovy);
        }
    }
    ImGui::EndGroup();

    ImGui::Separator();

        // Image area: size drives our render target size
        ImVec2 avail = ImGui::GetContentRegionAvail();
        int w = (int)std::max(4.0f, avail.x);
        int h = (int)std::max(4.0f, avail.y);
        cv.wantW = w;
        cv.wantH = h;

        // Show the last rendered texture (flip Y: GL is bottom-up)
        if (cv.rtColor) {
            ImGui::Image((ImTextureID)(intptr_t)cv.rtColor, avail, ImVec2(0,1), ImVec2(1,0));
        } else {
            // first frame before we rendered anything: just reserve space
            ImGui::Dummy(avail);
        }

}
ImGui::End();


    ++i;
}


// publish bottom for right stack
{
    ImVec2 p  = ImGui::GetWindowPos();
    ImVec2 sz = ImGui::GetWindowSize();
    gRightStackBottomY = std::max(gRightStackBottomY, p.y + sz.y);
}

ImGui::End(); // ===== end Cameras window =====


    // ===== Move window (right stack) =====
    if (gMove.active) {
        #ifdef IMGUI_HAS_VIEWPORT
                ImGui::SetNextWindowViewport(mainvp->ID);
                ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x - 10, gRightStackBottomY + 6),
                                        ImGuiCond_Always, ImVec2(1,0));
        #else
                ImGui::SetNextWindowPos(ImVec2(FBW-10, gRightStackBottomY + 6), ImGuiCond_Always, ImVec2(1,0));
        #endif
                ImGui::Begin("MOVE/COPY", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);

        // Move Object
        const char* objItems[] = {
            "Bodies", "Faces", "Points",
            "Viewport Anchors",
            "Cameras",
            "Components (soon)", "Joints (soon)"
        };

        int objIdx = (int)gMove.obj;
        ImGui::Combo("Move Object", &objIdx, objItems, IM_ARRAYSIZE(objItems));
        gMove.obj = (MoveObj)objIdx;

        // Selection summary
        {
                    int selCount = selectionCount();
                    char selBuf[64];
                    std::snprintf(selBuf, sizeof(selBuf), "%d selected", selCount);
                    ImGui::TextColored(ImVec4(0.2f,0.6f,1.f,1.f), selBuf);
        }
        ImGui::SameLine();
        if (ImGui::Button("x")) {
            gSelectedCube   = -1;
            gSelectedMesh   = -1;   // NEW: clear imported-mesh selection
            gSelectedAnchor = -1;
            gMove.active    = false;
            gMove.pickLocked = false; // <-- clear the point/face lock
            gSelAnchors.clear();
            gSelCams.clear();
            gSelectedCam = -1;
            gSelectedAnchor = -1;
        }



        // Move Type
        const char* typeItems[] = { "Free move", "Translate", "Rotate", "Point to Point (soon)", "Point to Position (soon)" };
        int typeIdx = (int)gMove.type; ImGui::Combo("Move Type", &typeIdx, typeItems, IM_ARRAYSIZE(typeItems)); gMove.type = (MoveType)typeIdx;

        if (gMove.obj != MoveObj::ViewportAnchors) {
            // Set Pivot (face center or point). When pressed we capture next click.
            static bool pickingPivot = false;
            if (ImGui::Button("Set Pivot")) pickingPivot = true;
            if (pickingPivot && !gMove.pickLocked && ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ioCaptures()) {
                // Compute ray, intersect visible cube faces; choose hit point
                double mx,my; glfwGetCursorPos(Win,&mx,&my);
                glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
                glm::mat4 V = Cam.view();
                glm::mat4 invPV = glm::inverse(P * V);
                glm::vec3 ro = glm::vec3(glm::inverse(V)[3]);  glm::vec3 rd = screenToRay(float(mx), float(my), invPV);

                // Intersect with actual cube surface by casting against planes of local faces and taking nearest >0
                const CubeState& C = activeCube();
                glm::mat4 M = glm::translate(glm::mat4(1.f), C.pos) * glm::mat4_cast(C.rot) * glm::scale(glm::mat4(1.f), glm::vec3(C.scale));
                glm::mat4 invM = glm::inverse(M);
                glm::vec3 rO = glm::vec3(invM * glm::vec4(ro,1));
                glm::vec3 rD = glm::normalize(glm::vec3(invM * glm::vec4(rd,0)));


                // slab t-range
                glm::vec3 tmin = (-0.5f - rO) / rD;
                glm::vec3 tmax = ( 0.5f - rO) / rD;
                glm::vec3 t0 = glm::min(tmin, tmax);
                glm::vec3 t1 = glm::max(tmin, tmax);
                float tEnter = std::max({t0.x, t0.y, t0.z});
                float tExit  = std::min({t1.x, t1.y, t1.z});
                if (tEnter <= tExit && tExit > 0.f) {
                    glm::vec3 pLocal = rO + tEnter * rD; // visible hit in unscaled local space
                    // Set pivot local position (NO division by scale)
                    // Compute pivot in LOCAL space (unscaled local coordinates)
                    if (gMove.obj == MoveObj::Points) {
                        // --- POINTS: snap to nearest cube vertex (±0.5 on each axis) ---
                        glm::vec3 v = pLocal;
                        glm::vec3 snapped(
                            (v.x >= 0.f ? 0.5f : -0.5f),
                            (v.y >= 0.f ? 0.5f : -0.5f),
                            (v.z >= 0.f ? 0.5f : -0.5f)
                        );
                        gMove.pivot.localPos   = snapped;
                        gMove.pivot.localBasis = glm::mat3(1.0f);
                    } else if (gMove.obj == MoveObj::Faces) {
                        // --- FACES: center of the hit face (dominant axis of |pLocal|) ---
                        glm::vec3 absP = glm::abs(pLocal);
                        glm::vec3 nLocal(0.f);
                        glm::vec3 center(0.f);

                        if (absP.x > absP.y && absP.x > absP.z) {
                            nLocal = glm::vec3((pLocal.x > 0.f) ? 1.f : -1.f, 0.f, 0.f);
                            center = glm::vec3((pLocal.x > 0.f) ? 0.5f : -0.5f, 0.f, 0.f);
                        } else if (absP.y > absP.x && absP.y > absP.z) {
                            nLocal = glm::vec3(0.f, (pLocal.y > 0.f) ? 1.f : -1.f, 0.f);
                            center = glm::vec3(0.f, (pLocal.y > 0.f) ? 0.5f : -0.5f, 0.f);
                        } else {
                            nLocal = glm::vec3(0.f, 0.f, (pLocal.z > 0.f) ? 1.f : -1.f);
                            center = glm::vec3(0.f, 0.f, (pLocal.z > 0.f) ? 0.5f : -0.5f);
                        }

                        gMove.pivot.localPos   = center;
                        gMove.pivot.localBasis = faceFrame(nLocal);
                    } else {
                        // Bodies/etc: leave as generic hit
                        gMove.pivot.localPos   = pLocal;
                        gMove.pivot.localBasis = glm::mat3(1.0f);
                    }

                    // lock the pick until cleared (Esc / X / OK-Cancel)
                    gMove.pickLocked = true;

                }
                pickingPivot = false;
            }
        }

        // --- Debounced numeric fields ---
        float m2u = 1.0f / unitToMeters(gUnit);
        ImGui::Separator();
        ImGui::Text("Distances (%s)", kUnitLabels[(int)gUnit]);

        auto debouncedFloat = [&](const char* label, float& v)->bool{
            bool changed = ImGui::InputFloat(label, &v, 0, 0, "%.3f");
            if (changed) { gMove.lastEditTime = ImGui::GetTime(); gMove.editedThisFrame = true; }
            return changed;
        };

        debouncedFloat("X Distance", gMove.ui_dx);
        debouncedFloat("Y Distance", gMove.ui_dy);
        debouncedFloat("Z Distance", gMove.ui_dz);

        ImGui::Text("Angles (deg)");
        debouncedFloat("X Angle", gMove.ui_ax);
        debouncedFloat("Y Angle", gMove.ui_ay);
        debouncedFloat("Z Angle", gMove.ui_az);

// --- Clone option (preview a copy; OK commits; Cancel deletes) ---
if (ImGui::Checkbox("Clone", &gMove.cloneChecked)) {
    if (gMove.cloneChecked) {
if (anchorMode()) {
            // NEW: multi-select anchors
            if (!gSelAnchors.empty()) {
                gMove.backupSelAnchors = gSelAnchors;
                gMove.cloneAnchorIdxs.clear();

                std::vector<int> newSel;
                newSel.reserve(gSelAnchors.size());

                for (int ai : gSelAnchors) {
                    if (ai < 0 || ai >= (int)gAnchors.size()) continue;
                    Anchor a = gAnchors[ai];
                    a.name += " (copy)";
                    gAnchors.push_back(a);
                    int ni = (int)gAnchors.size() - 1;
                    newSel.push_back(ni);
                    gMove.cloneAnchorIdxs.push_back(ni);
                }

                if (!newSel.empty()) {
                    gSelAnchors = newSel;
                    gSelectedAnchor = newSel.front();
                    // Snapshot at creation time (use first as reference)
                    gMove.snapPos = gAnchors[gSelectedAnchor].pos;
                    gMove.snapRot = glm::quat(1,0,0,0);
                    gMove.cloneAnchorIdx = -1; // ignore legacy single field
                }
            }
            // legacy single-anchor path
            else if (gSelectedAnchor >= 0) {
                Anchor a = gAnchors[gSelectedAnchor];
                a.name += " (copy)";
                gAnchors.push_back(a);
                gMove.cloneAnchorIdx = (int)gAnchors.size() - 1;
                gSelectedAnchor = gMove.cloneAnchorIdx;
                gMove.snapPos = gAnchors[gSelectedAnchor].pos;
                gMove.snapRot = glm::quat(1,0,0,0);
            }
        }
        else if (cameraMode()) {
            // NEW: multi-select cameras
            if (!gSelCams.empty()) {
                gMove.backupSelCams = gSelCams;
                gMove.cloneCamIdxs.clear();

                std::vector<int> newSel;
                newSel.reserve(gSelCams.size());

                for (int ci : gSelCams) {
                    if (ci < 0 || ci >= (int)gSceneCams.size()) continue;
                    // restore original to snapshot (like legacy single path) before cloning
                    if (gMove.snapTarget == MoveState::SnapTarget::Camera &&
                        gMove.snapCamIdx == ci) {
                        gSceneCams[ci].pos = gMove.snapPos;
                        gSceneCams[ci].rot = gMove.snapRot;
                    }
                    SceneCam c = gSceneCams[ci];
                    c.name += " (copy)";
                    gSceneCams.push_back(c);
                    int ni = (int)gSceneCams.size() - 1;
                    newSel.push_back(ni);
                    gMove.cloneCamIdxs.push_back(ni);
                }

                if (!newSel.empty()) {
                    gSelCams  = newSel;
                    gSelectedCam = newSel.front();
                    gMove.cloneCamIdx = -1; // ignore legacy single field
                }
            }
            // legacy single-camera path
            else {
                int origIdx = (gMove.snapTarget == MoveState::SnapTarget::Camera && gMove.snapCamIdx >= 0)
                              ? gMove.snapCamIdx : gSelectedCam;
                if (origIdx >= 0 && origIdx < (int)gSceneCams.size()) {
                    gSceneCams[origIdx].pos = gMove.snapPos;
                    gSceneCams[origIdx].rot = gMove.snapRot;

                    SceneCam c = gSceneCams[origIdx];
                    c.name += " (copy)";
                    gSceneCams.push_back(c);
                    gMove.cloneCamIdx = (int)gSceneCams.size() - 1;
                    gSelectedCam = gMove.cloneCamIdx;
                }
            }
        }
else { // Bodies
            // Multi-MESH cloning takes precedence if multiple meshes are selected
            if (!gSelectedMeshes.empty()) {
                // Backup original selection so we can restore on uncheck/cancel
                gMove.backupSelectedMeshes = gSelectedMeshes;
                gMove.cloneMeshIdxs.clear();

                std::vector<int> newSel;
                newSel.reserve(gSelectedMeshes.size());

                for (int mi : gSelectedMeshes) {
                    if (mi < 0 || mi >= (int)gMeshes.size()) continue;

                    // If this mesh was the one snapshotted at move start, restore it to its snapshot
                    if (gMove.snapTarget == MoveState::SnapTarget::Mesh &&
                        gMove.snapMeshIdx == mi)
                    {
                        gMeshes[mi].pos = gMove.snapPos;
                        gMeshes[mi].rot = gMove.snapRot;
                    }

                    ImportedMesh m = gMeshes[mi];
                    m.name += " (copy)";
                    gMeshes.push_back(m);
                    int ni = (int)gMeshes.size() - 1;
                    gMove.cloneMeshIdxs.push_back(ni);
                    newSel.push_back(ni);
                }

                if (!newSel.empty()) {
                    // Replace selection with new clones, keep gizmo on first
                    gSelectedMeshes = newSel;
                    gSelectedMesh = newSel.front();
                    gMove.primaryMeshForGizmo = gSelectedMesh;

                    // Clear cube selection
                    gSelectedCube  = -1;
                    gCube.selected = false;
                    gSelCubes.clear();

                    // Clear legacy single-mesh scratch
                    gMove.cloneMeshIdx = -1;
                }
            }
            // Single-mesh cloning (legacy path) when only one mesh is selected
            else if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
                const int origIdx = gSelectedMesh;

                if (gMove.snapTarget == MoveState::SnapTarget::Mesh &&
                    gMove.snapMeshIdx == origIdx)
                {
                    gMeshes[origIdx].pos = gMove.snapPos;
                    gMeshes[origIdx].rot = gMove.snapRot;
                }

                ImportedMesh m = gMeshes[origIdx];
                m.name += " (copy)";
                gMeshes.push_back(m);
                gMove.cloneMeshIdx = (int)gMeshes.size() - 1;

                // Select the temp clone for preview
                gSelectedMesh = gMove.cloneMeshIdx;

                // Make sure no cube is “active” during mesh clone preview
                gSelectedCube  = -1;
                gCube.selected = false;
                gSelCubes.clear();
            }
            // Multi-CUBE cloning (0 = main cube, 1..N = extras)
            else if (!gSelCubes.empty()) {
                gMove.backupSelCubes = gSelCubes;
                gMove.cloneCubeSelIdxs.clear();

                std::vector<int> newSel;
                newSel.reserve(gSelCubes.size());

                for (int sel : gSelCubes) {
                    const CubeState* src = nullptr;
                    if (sel == 0) src = &gCube;
                    else if (sel > 0 && sel - 1 < (int)gExtraCubes.size()) src = &gExtraCubes[sel - 1];
                    else src = &gCube;

                    CubeState c = *src;
                    gExtraCubes.push_back(c);
                    int newSelId = (int)gExtraCubes.size(); // selection id 1..N
                    newSel.push_back(newSelId);
                    gMove.cloneCubeSelIdxs.push_back(newSelId);
                }

                if (!newSel.empty()) {
                    gSelCubes = newSel;
                    gSelectedCube = newSel.front();
                    gCube.selected = (gSelectedCube == 0);

                    // Clear mesh selection
                    gSelectedMeshes.clear();
                    gSelectedMesh = -1;

                    // Ensure we are not in temp clone mode
                    gMove.cloneActive = false;
                }
            }
            // Fallback: single-cube temp preview if nothing mesh-like is selected
            else {
                CubeState* orig = nullptr;
                if      (gMove.snapCubeSel == 0) orig = &gCube;
                else if (gMove.snapCubeSel > 0 && gMove.snapCubeSel - 1 < (int)gExtraCubes.size())
                    orig = &gExtraCubes[gMove.snapCubeSel - 1];
                else
                    orig = &gCube;

                orig->pos = gMove.snapPos;
                orig->rot = gMove.snapRot;

                gMove.cloneCube   = *orig;
                gMove.cloneActive = true;

                gSelectedMesh   = -1;
                gSelectedCube   = -1;
                gCube.selected  = false;
                gSelCubes.clear();
            }
        }
    } else {
        // Unchecked before finalizing: delete temp clone and reselect the original object
 if (anchorMode()) {
            if (!gMove.cloneAnchorIdxs.empty()) {
                auto idxs = gMove.cloneAnchorIdxs;
                std::sort(idxs.begin(), idxs.end(), std::greater<int>());
                for (int i : idxs) {
                    if (i >= 0 && i < (int)gAnchors.size()) {
                        gAnchors.erase(gAnchors.begin() + i);
                    }
                }
                gSelAnchors = gMove.backupSelAnchors;
                gSelectedAnchor = gSelAnchors.empty() ? -1 : gSelAnchors.front();
                gMove.cloneAnchorIdxs.clear();
                gMove.backupSelAnchors.clear();
                gMove.cloneAnchorIdx = -1;
            } else {
                if (gMove.cloneAnchorIdx >= 0 && gMove.cloneAnchorIdx < (int)gAnchors.size()) {
                    gAnchors.erase(gAnchors.begin() + gMove.cloneAnchorIdx);
                }
                gMove.cloneAnchorIdx = -1;
                gSelectedAnchor = (gMove.snapTarget == MoveState::SnapTarget::Anchor) ? gMove.snapAnchorIdx : -1;
            }
        }
        else if (cameraMode()) {
            if (!gMove.cloneCamIdxs.empty()) {
                auto idxs = gMove.cloneCamIdxs;
                std::sort(idxs.begin(), idxs.end(), std::greater<int>());
                for (int i : idxs) {
                    if (i >= 0 && i < (int)gSceneCams.size()) {
                        gSceneCams.erase(gSceneCams.begin() + i);
                    }
                }
                gSelCams = gMove.backupSelCams;
                gSelectedCam = gSelCams.empty() ? -1 : gSelCams.front();
                gMove.cloneCamIdxs.clear();
                gMove.backupSelCams.clear();
                gMove.cloneCamIdx = -1;
            } else {
                if (gMove.cloneCamIdx >= 0 && gMove.cloneCamIdx < (int)gSceneCams.size()) {
                    gSceneCams.erase(gSceneCams.begin() + gMove.cloneCamIdx);
                }
                gMove.cloneCamIdx = -1;
                if (gMove.snapTarget == MoveState::SnapTarget::Camera)
                    gSelectedCam = gMove.snapCamIdx;
            }
        }
        else { // Bodies
            // --- Multi-mesh rollback (if we created several mesh clones) ---
            if (!gMove.cloneMeshIdxs.empty()) {
                // Erase clones in descending order to keep indices stable
                auto idxs = gMove.cloneMeshIdxs;
                std::sort(idxs.begin(), idxs.end(), std::greater<int>());
                for (int i : idxs) {
                    if (i >= 0 && i < (int)gMeshes.size()) {
                        gMeshes.erase(gMeshes.begin() + i);
                    }
                }
                // Restore original mesh selection
                gSelectedMeshes = gMove.backupSelectedMeshes;
                if (!gSelectedMeshes.empty()) {
                    gSelectedMesh = gSelectedMeshes.front();
                    gMove.primaryMeshForGizmo = gSelectedMesh;
                } else {
                    gSelectedMesh = -1;
                    gMove.primaryMeshForGizmo = -1;
                }
                gMove.cloneMeshIdxs.clear();
                gMove.backupSelectedMeshes.clear();
                gMove.cloneMeshIdx = -1; // clear legacy single
            }
            // --- Single-mesh rollback (your legacy path) ---
            else if (gMove.cloneMeshIdx >= 0) {
                if (gMove.cloneMeshIdx < (int)gMeshes.size())
                    gMeshes.erase(gMeshes.begin() + gMove.cloneMeshIdx);

                if (gMove.snapTarget == MoveState::SnapTarget::Mesh &&
                    gMove.snapMeshIdx >= 0 && gMove.snapMeshIdx < (int)gMeshes.size())
                {
                    gSelectedMesh = gMove.snapMeshIdx;
                    gMeshes[gSelectedMesh].pos = gMove.snapPos;
                    gMeshes[gSelectedMesh].rot = gMove.snapRot;
                } else {
                    gSelectedMesh = -1;
                }

                gMove.cloneMeshIdx = -1;
            }

            // --- Multi-cube rollback (if we created several cube clones) ---
            if (!gMove.cloneCubeSelIdxs.empty()) {
                // Convert selection IDs to extra indices and erase descending
                std::vector<int> extraIdxs;
                extraIdxs.reserve(gMove.cloneCubeSelIdxs.size());
                for (int selId : gMove.cloneCubeSelIdxs) {
                    if (selId > 0) extraIdxs.push_back(selId - 1);
                }
                std::sort(extraIdxs.begin(), extraIdxs.end(), std::greater<int>());
                for (int ei : extraIdxs) {
                    if (ei >= 0 && ei < (int)gExtraCubes.size())
                        gExtraCubes.erase(gExtraCubes.begin() + ei);
                }

                // Restore original cube selection
                gSelCubes = gMove.backupSelCubes;
                if (!gSelCubes.empty()) {
                    gSelectedCube = gSelCubes.front();
                    gCube.selected = (gSelectedCube == 0);
                } else {
                    gSelectedCube = -1;
                    gCube.selected = false;
                }

                gMove.cloneCubeSelIdxs.clear();
                gMove.backupSelCubes.clear();
                gMove.cloneActive = false;
            }
            // --- Single-cube temp preview rollback (unchanged) ---
            else if (gMove.cloneActive) {
                gMove.cloneActive = false;
            }
 else {
                // Fallback: existing cube path (unchanged)
                gMove.cloneActive = false;
                if (gMove.snapTarget == MoveState::SnapTarget::Cube) {
                    gSelectedCube  = gMove.snapCubeSel; // 0 = main, >0 = extras[i-1]
                    gCube.selected = (gSelectedCube == 0);
                    gSelCubes.clear();
                    if (gSelectedCube >= 0) gSelCubes.push_back(gSelectedCube);
                }
            }
        }
    }
}




        // Apply debounced edits after 0.25s idle
        if (gMove.editedThisFrame && (ImGui::GetTime() - gMove.lastEditTime) > 0.25) {
            const glm::mat3 B = currentPivotWorldBasis();
            const glm::vec3 d = B * glm::vec3(gMove.ui_dx * unitToMeters(gUnit),
                                               gMove.ui_dy * unitToMeters(gUnit),
                                               gMove.ui_dz * unitToMeters(gUnit));

            // Rotation (world-space), identity if all angles are zero
            glm::quat q = glm::quat(1,0,0,0);
            if (!anchorMode()) {
                const glm::quat qx = glm::angleAxis(glm::radians(gMove.ui_ax), glm::normalize(B[0]));
                const glm::quat qy = glm::angleAxis(glm::radians(gMove.ui_ay), glm::normalize(B[1]));
                const glm::quat qz = glm::angleAxis(glm::radians(gMove.ui_az), glm::normalize(B[2]));
                q = qz * qy * qx;
            }

            const glm::vec3 pw = currentPivotWorldPos();

            // Bodies: support multi-mesh numeric edits
            const bool bodiesMode = (gMove.obj == MoveObj::Bodies);

            if (bodiesMode && !gSelectedMeshes.empty()) {
                // Apply to every selected mesh using its own snapshot
                for (int mi : gSelectedMeshes) {
                    if (mi < 0 || mi >= (int)gMeshes.size()) continue;
                    const glm::vec3 snap = (gMove.multiSnapPos.count(mi) ? gMove.multiSnapPos[mi] : gMeshes[mi].pos);
                    glm::vec3 pos = pw + (q * (snap - pw)) + d;
                    gMeshes[mi].pos = pos;
                }
                // (Optional) no mesh rotation via numeric angles here; add if you expose mesh rotations.
            if (bodiesMode && gMove.cloneActive && gSelectedMeshes.empty() && gSelectedMesh < 0) {
                    // Apply numeric transform to the temp clone relative to the frozen pivot
                    glm::vec3 pos = pw + (q * (gMove.snapPos - pw)) + d;
                    gMove.cloneCube.pos = pos;
                    if (gMove.ui_ax != 0 || gMove.ui_ay != 0 || gMove.ui_az != 0) {
                        gMove.cloneCube.rot = glm::normalize(q * gMove.snapRot);
                    }
                }

            } else {
                // Single target (cube / camera / anchor, or single body)
                if (glm::vec3* P = currentPosPtr()) {
                    *P = pw + (q * (gMove.snapPos - pw)) + d;   // <-- compose rotation and translation correctly
                }
                if (!anchorMode()) {
                    if (glm::quat* R = currentRotPtr()) {
                        *R = glm::normalize(q * gMove.snapRot);
                    }
                }
            }

            gMove.editedThisFrame = false;
        }
ImGui::Separator();
if (ImGui::Button("OK", ImVec2(120,0))) {
    if (anchorMode()) {
        if (!gMove.cloneAnchorIdxs.empty()) {
            // Keep clones & current selection; just clear bookkeeping
            gMove.cloneAnchorIdxs.clear();
            gMove.backupSelAnchors.clear();
            gMove.cloneAnchorIdx = -1; // legacy field
        } else {
            gMove.cloneAnchorIdx = -1;
        }
    } else if (cameraMode()) {
        if (!gMove.cloneCamIdxs.empty()) {
            gMove.cloneCamIdxs.clear();
            gMove.backupSelCams.clear();
            gMove.cloneCamIdx = -1; // legacy field
        } else {
            gMove.cloneCamIdx = -1;
        }
    }
    else if (!gMove.cloneMeshIdxs.empty()) {
        if (!gSelectedMeshes.empty()) {
            gSelectedMesh = gSelectedMeshes.front();
            gMove.primaryMeshForGizmo = gSelectedMesh;
        }
        gMove.cloneMeshIdxs.clear();
        gMove.backupSelectedMeshes.clear();
        gMove.cloneMeshIdx = -1;
    }
    else if (!gMove.cloneCubeSelIdxs.empty()) {
        if (!gSelCubes.empty()) {
            gSelectedCube = gSelCubes.front();
            gCube.selected = (gSelectedCube == 0);
        }
        gMove.cloneCubeSelIdxs.clear();
        gMove.backupSelCubes.clear();
        gMove.cloneActive = false;
    }
    else if (gMove.cloneMeshIdx >= 0) {
        gSelectedMesh      = gMove.cloneMeshIdx;
        gMove.cloneMeshIdx = -1;
    } else if (gMove.cloneActive) {
        gExtraCubes.push_back(gMove.cloneCube);
        gMove.cloneActive = false;
        gSelectedCube = (int)gExtraCubes.size();
        gSelCubes.clear();
        gSelCubes.push_back(gSelectedCube);
        gCube.selected = (gSelectedCube == 0);
    }

    // ... your existing cleanup (unchanged)
    gMove.active = false;
    gMove.multiSnapPos.clear();
    gMove.multiSnapAnchorPos.clear(); // NEW
    gMove.multiSnapCamPos.clear();    // NEW
    gMove.multiSnapCamRot.clear();    // NEW

    gMove.primaryMeshForGizmo = -1;
    gMove.pickLocked = false;
    gMove.dragging = false;
    glfwSwapInterval(1);
    gMove.hot = MoveState::None;
    gMove.lastAxis = -1;
    gMove.axisSnapPending = false;
}

ImGui::SameLine();
if (ImGui::Button("Cancel", ImVec2(120,0))) {
    if (anchorMode()) {
        if (!gMove.cloneAnchorIdxs.empty()) {
            // Roll back created clones
            auto idxs = gMove.cloneAnchorIdxs;
            std::sort(idxs.begin(), idxs.end(), std::greater<int>());
            for (int i : idxs)
                if (i >= 0 && i < (int)gAnchors.size())
                    gAnchors.erase(gAnchors.begin() + i);

            // Restore original anchor selection
            gSelAnchors     = gMove.backupSelAnchors;
            gSelectedAnchor = gSelAnchors.empty() ? -1 : gSelAnchors.front();

            gMove.cloneAnchorIdxs.clear();
            gMove.backupSelAnchors.clear();
            gMove.cloneAnchorIdx = -1;
        } else {
            // No clones: restore ALL snapshotted anchors
            for (const auto& kv : gMove.multiSnapAnchorPos) {
                int ai = kv.first;
                if (ai >= 0 && ai < (int)gAnchors.size())
                    gAnchors[ai].pos = kv.second;
            }
        }
        // Clear anchor snapshots
        gMove.multiSnapAnchorPos.clear();
    }
    else if (cameraMode()) {
        if (!gMove.cloneCamIdxs.empty()) {
            // Roll back created camera clones
            auto idxs = gMove.cloneCamIdxs;
            std::sort(idxs.begin(), idxs.end(), std::greater<int>());
            for (int i : idxs)
                if (i >= 0 && i < (int)gSceneCams.size())
                    gSceneCams.erase(gSceneCams.begin() + i);

            // Restore original camera selection
            gSelCams     = gMove.backupSelCams;
            gSelectedCam = gSelCams.empty() ? -1 : gSelCams.front();

            gMove.cloneCamIdxs.clear();
            gMove.backupSelCams.clear();
            gMove.cloneCamIdx = -1;
        } else {
            // No clones: restore ALL snapshotted cameras (pos & rot)
            if (!gMove.multiSnapCamPos.empty()) {
                for (const auto& kv : gMove.multiSnapCamPos) {
                    int ci = kv.first;
                    if (ci >= 0 && ci < (int)gSceneCams.size())
                        gSceneCams[ci].pos = kv.second;
                }
                for (const auto& kv : gMove.multiSnapCamRot) {
                    int ci = kv.first;
                    if (ci >= 0 && ci < (int)gSceneCams.size())
                        gSceneCams[ci].rot = kv.second;
                }
            } else if (gMove.snapTarget == MoveState::SnapTarget::Camera &&
                       gMove.snapCamIdx >= 0 && gMove.snapCamIdx < (int)gSceneCams.size()) {
                // Fallback: single-select restore if multi maps were never populated
                gSceneCams[gMove.snapCamIdx].pos = gMove.snapPos;
                gSceneCams[gMove.snapCamIdx].rot = gMove.snapRot;
                gSelectedCam = gMove.snapCamIdx;
                       }
        }
        // Clear camera snapshots
        gMove.multiSnapCamPos.clear();
        gMove.multiSnapCamRot.clear();
    }

else {
    // Bodies
    // NEW: multi-mesh rollback
    if (!gMove.cloneMeshIdxs.empty()) {
        auto idxs = gMove.cloneMeshIdxs;
        std::sort(idxs.begin(), idxs.end(), std::greater<int>());
        for (int i : idxs) {
            if (i >= 0 && i < (int)gMeshes.size())
                gMeshes.erase(gMeshes.begin() + i);
        }
        gSelectedMeshes = gMove.backupSelectedMeshes;
        if (!gSelectedMeshes.empty()) {
            gSelectedMesh = gSelectedMeshes.front();
            gMove.primaryMeshForGizmo = gSelectedMesh;
        } else {
            gSelectedMesh = -1;
            gMove.primaryMeshForGizmo = -1;
        }
        gMove.cloneMeshIdxs.clear();
        gMove.backupSelectedMeshes.clear();
        gMove.cloneMeshIdx = -1;
    }
    // existing single-mesh rollback
    else if (gMove.cloneMeshIdx >= 0) {
        if (gMove.cloneMeshIdx < (int)gMeshes.size())
            gMeshes.erase(gMeshes.begin() + gMove.cloneMeshIdx);

        if (gMove.snapTarget == MoveState::SnapTarget::Mesh &&
            gMove.snapMeshIdx >= 0 && gMove.snapMeshIdx < (int)gMeshes.size())
        {
            gSelectedMesh = gMove.snapMeshIdx;
            gMeshes[gSelectedMesh].pos = gMove.snapPos;
            gMeshes[gSelectedMesh].rot = gMove.snapRot;
        } else {
            gSelectedMesh = -1;
        }
        gMove.cloneMeshIdx = -1;
    }

    // NEW: multi-cube rollback
    if (!gMove.cloneCubeSelIdxs.empty()) {
        std::vector<int> extraIdxs;
        extraIdxs.reserve(gMove.cloneCubeSelIdxs.size());
        for (int selId : gMove.cloneCubeSelIdxs)
            if (selId > 0) extraIdxs.push_back(selId - 1);
        std::sort(extraIdxs.begin(), extraIdxs.end(), std::greater<int>());
        for (int ei : extraIdxs) {
            if (ei >= 0 && ei < (int)gExtraCubes.size())
                gExtraCubes.erase(gExtraCubes.begin() + ei);
        }

        // Restore original cube selection
        gSelCubes = gMove.backupSelCubes;
        if (!gSelCubes.empty()) {
            gSelectedCube = gSelCubes.front();
            gCube.selected = (gSelectedCube == 0);
        } else {
            gSelectedCube = -1;
            gCube.selected = false;
        }

        gMove.cloneCubeSelIdxs.clear();
        gMove.backupSelCubes.clear();
        gMove.cloneActive = false;
    }
    // existing single-cube rollback
    else if (gMove.cloneActive) {
        gMove.cloneActive = false; // drop temp cube clone
    }
}
    // Clear UI deltas and angles so nothing re-applies after Cancel
    gMove.ui_dx = gMove.ui_dy = gMove.ui_dz = 0.f;
    gMove.ui_ax = gMove.ui_ay = gMove.ui_az = 0.f;

    // End session & clear snapshots
    gMove.active = false;
    gMove.dragging = false;
    glfwSwapInterval(1); // restore V-Sync after drag (optional)

    gMove.multiSnapPos.clear();
    gMove.multiSnapAnchorPos.clear();
    gMove.multiSnapCamPos.clear();
    gMove.multiSnapCamRot.clear();

    gMove.primaryMeshForGizmo = -1;
    gMove.pickLocked = false;

    gMove.hot = MoveState::None;
    gMove.lastAxis = -1;
    gMove.axisSnapPending = false;
}




        // publish bottom for cube placement
        ImVec2 aPos  = ImGui::GetWindowPos();
        ImVec2 aSize = ImGui::GetWindowSize();
        const float moveBottom = aPos.y + aSize.y;
        gRightStackBottomY = std::max(gRightStackBottomY, moveBottom);

        ImGui::End();
    }


    // ===== Units dropdown (bottom-right) =====
    {
        #ifdef IMGUI_HAS_VIEWPORT
                    ImGui::SetNextWindowViewport(mainvp->ID);
                    ImGui::SetNextWindowPos(ImVec2(mainvp->Pos.x + mainvp->Size.x - 10,
                                                   mainvp->Pos.y + mainvp->Size.y - 10),
                                            ImGuiCond_Always, ImVec2(1,1));
        #else
                    ImGui::SetNextWindowPos(ImVec2(FBW-10, FBH-10), ImGuiCond_Always, ImVec2(1,1));
        #endif
                    ImGui::Begin("Units", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize |
                                                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize);

        int u = (int)gUnit;
        ImGui::SetNextItemWidth(90);
        ImGui::Combo("##units", &u, kUnitLabels, IM_ARRAYSIZE(kUnitLabels));
        gUnit = (Unit)u;
        ImGui::End();
    }
}
//---------------------------------- Input callbacks ----------------------------
static bool ioCaptures(){ ImGuiIO& io=ImGui::GetIO(); return io.WantCaptureMouse || io.WantCaptureKeyboard; }
static void cursorCB(GLFWwindow* w, double x, double y) {
    double dx = x - lastX, dy = y - lastY;
    lastX = x; lastY = y;
    // If we are waiting to see if this is a drag, promote to DRAG on small motion
    if (cubePendingClick && !cubeDragging) {
        const double ddx = x - cubePressX;
        const double ddy = y - cubePressY;
        if (ddx*ddx + ddy*ddy >= 9.0) {      // 3px threshold
            cubeDragging = true;
            cubePendingClick = false;        // no snap on release now
            cubeDragLastX = x; cubeDragLastY = y;
        }
    }

    // Nav-cube drag always works (even if ImGui captures mouse)
    // SAFETY: if the left mouse button was released while dragging the nav cube,
    // stop the drag even if the release event didn't reach mouseBtnCB.
    if (cubeDragging && glfwGetMouseButton(Win, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS) {
        cubeDragging = false;
    }

    if (cubeDragging) {
        const float rot = 0.005f;
        double ddx = x - cubeDragLastX;
        double ddy = y - cubeDragLastY;
        cubeDragLastX = x; cubeDragLastY = y;

        Cam.yaw   += rot * ddx;                 // left/right -> yaw
        Cam.pitch += rot * ddy;                 // up/down   -> pitch
        Cam.pitch  = std::clamp(Cam.pitch, -1.55f, 1.55f);
        return; // consume while dragging the cube
    }

    // 2) Middle mouse: pan / orbit (works even when ImGui captures)
    bool mmb   = (glfwGetMouseButton(Win, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    bool shift = (glfwGetKey(Win, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS) ||
                 (glfwGetKey(Win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    if (mmb) {
        if (shift) {
            // Orbit around the center
            const float rot = 0.005f;
            Cam.yaw   += rot * dx;
            Cam.pitch += rot * dy;
            Cam.pitch  = std::clamp(Cam.pitch, -1.55f, 1.55f);
        } else {
            // Pan the camera (viewport moves as specified)
            glm::mat4 V    = Cam.view();
            glm::mat4 invV = glm::inverse(V);
            glm::vec3 right= glm::normalize(glm::vec3(invV[0]));
            glm::vec3 up   = glm::normalize(glm::vec3(invV[1]));

            float span = std::max({ S.width_m, S.length_m, S.depth_m, 1.0f });
            float s = 0.0009f * span;                    // slower pan

            // Left drag => viewport right (model left): use -dx
            // Down drag => viewport down (model up):   use +dy
            Cam.pan += (-float(dx)) * s * right + ( float(dy)) * s * up;
        }
        return;
    }

    // For everything else, respect ImGui capture
    if (ioCaptures()) return;

    const float rot = 0.005f;
    const float pan = 0.01f;

    // --- existing Shift+MMB orbit / MMB pan below stays unchanged ---
    const bool shiftHeld =
        glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
        glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    if (MMB && shiftHeld) {
        // If an anchor is focused, orbit around that anchor’s position
        if (gFocusedAnchor >= 0 && gFocusedAnchor < (int)gAnchors.size())
            Cam.pan = gAnchors[gFocusedAnchor].pos;

        Cam.yaw   += rot * dx;
        Cam.pitch += rot * dy;
        Cam.pitch  = std::clamp(Cam.pitch, -1.55f, 1.55f);
    }
    else if (MMB) {
        // (your fixed screen-space pan code here – unchanged)
        glm::vec3 fwd = glm::normalize(glm::vec3(
            -std::cos(Cam.pitch) * std::cos(Cam.yaw),
            -std::sin(Cam.pitch),
            -std::cos(Cam.pitch) * std::sin(Cam.yaw)
        ));
        glm::vec3 worldUp(0.f, 1.f, 0.f);
        glm::vec3 right = glm::cross(fwd, worldUp);
        if (glm::dot(right, right) < 1e-8f) right = glm::vec3(1,0,0);
        else right = glm::normalize(right);
        glm::vec3 upCam = glm::normalize(glm::cross(right, fwd));
        const float k = pan * Cam.dist * 0.05f;
        Cam.pan -= right * float(dx) * k;
        Cam.pan += upCam * float(dy) * k;
    }
// --- Move gizmo drag (LMB) ---
    // SAFETY: if the left mouse button was released while dragging the gizmo,
    // finalize the drag even if the release event didn't reach mouseBtnCB.
    if (gMove.dragging && glfwGetMouseButton(Win, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS) {
        int lastAxis = -1;
        if (gMove.hot == MoveState::AxisX) lastAxis = 0;
        else if (gMove.hot == MoveState::AxisY) lastAxis = 1;
        else if (gMove.hot == MoveState::AxisZ) lastAxis = 2;

        gMove.dragging = false;
        glfwSwapInterval(1);                         // match your release path
        gMove.axisSnapPending = (lastAxis != -1);    // one-shot snap after release
        gMove.lastAxis = lastAxis;
        gMove.hot = MoveState::None;
    }

    // Allow gizmo drags to update even if ImGui is capturing mouse (match nav-cube behavior)
    if (gMove.active && gMove.dragging) {
        // Build a fresh ray from cursor
        glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
        glm::mat4 V = Cam.view();
        glm::mat4 invPV = glm::inverse(P * V);
        glm::vec3 ro = glm::vec3(glm::inverse(V)[3]);
        glm::vec3 rd = screenToRay(float(lastX), float(lastY), invPV);

        // Use the FROZEN drag frame (don’t recompute from moving target)
        const glm::vec3 pw0 = gMove.dragPivotStart;
        const glm::mat3 B0  = gMove.dragStartBasis;

        const float m2u = 1.0f / unitToMeters(gUnit); // meters -> user units


if (gMove.hot == MoveState::AxisX || gMove.hot == MoveState::AxisY || gMove.hot == MoveState::AxisZ) {
    const glm::vec3 ax = gMove.dragStartAxis;   // unit, frozen at mouse-down
    const glm::vec3 pw = gMove.dragStartPw;     // pivot at mouse-down
    const glm::vec3 n  = gMove.dragStartPlaneN; // constraint plane normal (frozen)

    // PASS 1: intersect current ray with the frozen plane through the frozen pivot
    glm::vec3 hit0;
    if (rayPlane(ro, rd, pw, n, hit0)) {
        const float t0 = glm::dot(hit0 - pw, ax);       // project hit onto axis
        const float d0 = t0 - gMove.dragStart_tAxis;    // first estimate of axis travel

        // Move the plane origin along the axis by the first estimate and refine
        const glm::vec3 pw1 = pw + ax * d0;

        glm::vec3 hit1;
        if (rayPlane(ro, rd, pw1, n, hit1)) {
            const float t1 = glm::dot(hit1 - pw, ax);
            const float d  = t1 - gMove.dragStart_tAxis; // refined axis travel
            const glm::vec3 deltaW = d * ax;
// Apply axis delta to all selected (baseline + deltaW)
bool anyApplied = false;

// Meshes
if (!gSelectedMeshes.empty()) {
    for (int mi : gSelectedMeshes) {
        if (mi < 0 || mi >= (int)gMeshes.size()) continue;
        const glm::vec3 base = gMove.multiSnapPos.count(mi) ? gMove.multiSnapPos[mi] : gMeshes[mi].pos;
        gMeshes[mi].pos = base + deltaW;
    }
    anyApplied = true;
} else if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
    const glm::vec3 base = gMove.multiSnapPos.count(gSelectedMesh) ? gMove.multiSnapPos[gSelectedMesh] : gMeshes[gSelectedMesh].pos;
    gMeshes[gSelectedMesh].pos = base + deltaW;
    anyApplied = true;
}

// Cubes
if (!gSelCubes.empty()) {
    for (int ci : gSelCubes) {
        if (ci == 0) {
            const glm::vec3 base = gMove.multiSnapCubePos.count(0) ? gMove.multiSnapCubePos[0] : gCube.pos;
            gCube.pos = base + deltaW;
        } else if (ci > 0) {
            const int idx = ci - 1;
            if (idx >= 0 && idx < (int)gExtraCubes.size()) {
                const glm::vec3 base = gMove.multiSnapCubePos.count(ci) ? gMove.multiSnapCubePos[ci] : gExtraCubes[idx].pos;
                gExtraCubes[idx].pos = base + deltaW;
            }
        }
    }
    anyApplied = true;
} else if (gSelectedCube == 0) {
    const glm::vec3 base = gMove.multiSnapCubePos.count(0) ? gMove.multiSnapCubePos[0] : gCube.pos;
    gCube.pos = base + deltaW;
    anyApplied = true;
} else if (gSelectedCube > 0 && (gSelectedCube-1) < (int)gExtraCubes.size()) {
    const int idx = gSelectedCube - 1;
    const glm::vec3 base = gMove.multiSnapCubePos.count(gSelectedCube) ? gMove.multiSnapCubePos[gSelectedCube] : gExtraCubes[idx].pos;
    gExtraCubes[idx].pos = base + deltaW;
    anyApplied = true;
}

// Anchors
if (!gSelAnchors.empty()) {
    for (int ai : gSelAnchors) {
        if (ai >= 0 && ai < (int)gAnchors.size()) {
            const glm::vec3 base = gMove.multiSnapAnchorPos.count(ai) ? gMove.multiSnapAnchorPos[ai] : gAnchors[ai].pos;
            gAnchors[ai].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
    const glm::vec3 base = gMove.multiSnapAnchorPos.count(gSelectedAnchor) ? gMove.multiSnapAnchorPos[gSelectedAnchor] : gAnchors[gSelectedAnchor].pos;
    gAnchors[gSelectedAnchor].pos = base + deltaW;
    anyApplied = true;
}

// Cameras
if (!gSelCams.empty()) {
    for (int ci : gSelCams) {
        if (ci >= 0 && ci < (int)gSceneCams.size()) {
            const glm::vec3 base = gMove.multiSnapCamPos.count(ci) ? gMove.multiSnapCamPos[ci] : gSceneCams[ci].pos;
            gSceneCams[ci].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
    const glm::vec3 base = gMove.multiSnapCamPos.count(gSelectedCam) ? gMove.multiSnapCamPos[gSelectedCam] : gSceneCams[gSelectedCam].pos;
    gSceneCams[gSelectedCam].pos = base + deltaW;
    anyApplied = true;
}

// Fallback (generic single target)
if (!anyApplied) {
    if (glm::vec3* Ppos = currentPosPtr())
        *Ppos = gMove.dragBasePos + deltaW;
}


            const glm::vec3 local = glm::transpose(B0) * deltaW; // B0 = frozen basis
            gMove.ui_dx = local.x * m2u;
            gMove.ui_dy = local.y * m2u;
            gMove.ui_dz = local.z * m2u;

            gMove.dragGrabW = hit1; // stays visually under the cursor
        } else {
            // Fallback to first-pass estimate if the refined plane is degenerate
            const glm::vec3 deltaW = d0 * ax;
// Apply axis delta to all selected (baseline + deltaW)  << REPLACE
bool anyApplied = false;


// Meshes
if (!gSelectedMeshes.empty()) {
    for (int mi : gSelectedMeshes) {
        if (mi < 0 || mi >= (int)gMeshes.size()) continue;
        const glm::vec3 base = gMove.multiSnapPos.count(mi) ? gMove.multiSnapPos[mi] : gMeshes[mi].pos;
        gMeshes[mi].pos = base + deltaW;
    }
    anyApplied = true;
} else if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
    const glm::vec3 base = gMove.multiSnapPos.count(gSelectedMesh) ? gMove.multiSnapPos[gSelectedMesh] : gMeshes[gSelectedMesh].pos;
    gMeshes[gSelectedMesh].pos = base + deltaW;
    anyApplied = true;
}

// Cubes
if (!gSelCubes.empty()) {
    for (int ci : gSelCubes) {
        if (ci == 0) {
            const glm::vec3 base = gMove.multiSnapCubePos.count(0) ? gMove.multiSnapCubePos[0] : gCube.pos;
            gCube.pos = base + deltaW;
        } else if (ci > 0) {
            const int idx = ci - 1;
            if (idx >= 0 && idx < (int)gExtraCubes.size()) {
                const glm::vec3 base = gMove.multiSnapCubePos.count(ci) ? gMove.multiSnapCubePos[ci] : gExtraCubes[idx].pos;
                gExtraCubes[idx].pos = base + deltaW;
            }
        }
    }
    anyApplied = true;
} else if (gSelectedCube == 0) {
    const glm::vec3 base = gMove.multiSnapCubePos.count(0) ? gMove.multiSnapCubePos[0] : gCube.pos;
    gCube.pos = base + deltaW;
    anyApplied = true;
} else if (gSelectedCube > 0 && (gSelectedCube-1) < (int)gExtraCubes.size()) {
    const int idx = gSelectedCube - 1;
    const glm::vec3 base = gMove.multiSnapCubePos.count(gSelectedCube) ? gMove.multiSnapCubePos[gSelectedCube] : gExtraCubes[idx].pos;
    gExtraCubes[idx].pos = base + deltaW;
    anyApplied = true;
}

// Anchors
if (!gSelAnchors.empty()) {
    for (int ai : gSelAnchors) {
        if (ai >= 0 && ai < (int)gAnchors.size()) {
            const glm::vec3 base = gMove.multiSnapAnchorPos.count(ai) ? gMove.multiSnapAnchorPos[ai] : gAnchors[ai].pos;
            gAnchors[ai].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
    const glm::vec3 base = gMove.multiSnapAnchorPos.count(gSelectedAnchor) ? gMove.multiSnapAnchorPos[gSelectedAnchor] : gAnchors[gSelectedAnchor].pos;
    gAnchors[gSelectedAnchor].pos = base + deltaW;
    anyApplied = true;
}

// Cameras
if (!gSelCams.empty()) {
    for (int ci : gSelCams) {
        if (ci >= 0 && ci < (int)gSceneCams.size()) {
            const glm::vec3 base = gMove.multiSnapCamPos.count(ci) ? gMove.multiSnapCamPos[ci] : gSceneCams[ci].pos;
            gSceneCams[ci].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
    const glm::vec3 base = gMove.multiSnapCamPos.count(gSelectedCam) ? gMove.multiSnapCamPos[gSelectedCam] : gSceneCams[gSelectedCam].pos;
    gSceneCams[gSelectedCam].pos = base + deltaW;
    anyApplied = true;
}

// Fallback (generic single target)
if (!anyApplied) {
    if (glm::vec3* Ppos = currentPosPtr())
        *Ppos = gMove.dragBasePos + deltaW;
}


            const glm::vec3 local = glm::transpose(B0) * deltaW;
            gMove.ui_dx = local.x * m2u;
            gMove.ui_dy = local.y * m2u;
            gMove.ui_dz = local.z * m2u;

            gMove.dragGrabW = hit0;
        }
    } else {
        // Last resort: fall back to line–line closest if the plane is parallel to the ray
        float tNow; glm::vec3 closest;
        if (closestOnAxis(ro, rd, pw, ax, tNow, closest)) {
            const float d = tNow - gMove.dragStart_tAxis;
            const glm::vec3 deltaW = d * ax;

// Apply axis delta to all selected (baseline + deltaW)  << REPLACE
bool anyApplied = false;

// Meshes
if (!gSelectedMeshes.empty()) {
    for (int mi : gSelectedMeshes) {
        if (mi < 0 || mi >= (int)gMeshes.size()) continue;
        glm::vec3 base = gMove.multiSnapPos.count(mi) ? gMove.multiSnapPos[mi] : gMeshes[mi].pos;
        gMeshes[mi].pos = base + deltaW;
    }
    anyApplied = true;
}

// Cubes
if (!gSelCubes.empty()) {
    for (int ci : gSelCubes) {
        if (ci == 0) {
            glm::vec3 base = gMove.multiSnapCubePos.count(0) ? gMove.multiSnapCubePos[0] : gCube.pos;
            gCube.pos = base + deltaW;
        } else if (ci > 0) {
            int idx = ci - 1;
            if (idx >= 0 && idx < (int)gExtraCubes.size()) {
                glm::vec3 base = gMove.multiSnapCubePos.count(ci) ? gMove.multiSnapCubePos[ci] : gExtraCubes[idx].pos;
                gExtraCubes[idx].pos = base + deltaW;
            }
        }
    }
    anyApplied = true;
}

// Anchors
if (!gSelAnchors.empty()) {
    for (int ai : gSelAnchors) {
        if (ai >= 0 && ai < (int)gAnchors.size()) {
            glm::vec3 base = gMove.multiSnapAnchorPos.count(ai) ? gMove.multiSnapAnchorPos[ai] : gAnchors[ai].pos;
            gAnchors[ai].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
    // Robust single selection fallback
    glm::vec3 base = gMove.multiSnapAnchorPos.count(gSelectedAnchor) ? gMove.multiSnapAnchorPos[gSelectedAnchor]
                                                                     : gAnchors[gSelectedAnchor].pos;
    gAnchors[gSelectedAnchor].pos = base + deltaW;
    anyApplied = true;
}

// Cameras (position only in translation path)
if (!gSelCams.empty()) {
    for (int ci : gSelCams) {
        if (ci >= 0 && ci < (int)gSceneCams.size()) {
            glm::vec3 base = gMove.multiSnapCamPos.count(ci) ? gMove.multiSnapCamPos[ci] : gSceneCams[ci].pos;
            gSceneCams[ci].pos = base + deltaW;
        }
    }
    anyApplied = true;
} else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
    glm::vec3 base = gMove.multiSnapCamPos.count(gSelectedCam) ? gMove.multiSnapCamPos[gSelectedCam]
                                                               : gSceneCams[gSelectedCam].pos;
    gSceneCams[gSelectedCam].pos = base + deltaW;
    anyApplied = true;
}

// Fallback to single current target (bodies path only)
if (!anyApplied) {
    if (glm::vec3* Ppos = currentPosPtr())
        *Ppos = gMove.dragBasePos + deltaW;
}


            const glm::vec3 local = glm::transpose(B0) * deltaW;
            gMove.ui_dx = local.x * m2u;
            gMove.ui_dy = local.y * m2u;
            gMove.ui_dz = local.z * m2u;

            gMove.dragGrabW = closest;
        }
        // else do nothing this frame
    }
}

    else if (gMove.hot == MoveState::PlaneXY || gMove.hot == MoveState::PlaneXZ || gMove.hot == MoveState::PlaneYZ) {
        // Use the frozen plane
        const glm::vec3 n = gMove.dragStartPlaneN;
        glm::vec3 hit;
        if (rayPlane(ro, rd, pw0, n, hit)) {
            glm::vec3 delta = hit - gMove.dragGrabW;
            // Constrain to plane
            delta -= glm::dot(delta, n) * n;

            // Apply plane delta to all selected (incremental)  << UPDATED
            bool anyApplied = false;

            // Meshes
            if (!gSelectedMeshes.empty()) {
                for (int mi : gSelectedMeshes) {
                    if (mi < 0 || mi >= (int)gMeshes.size()) continue;
                    gMeshes[mi].pos += delta;
                }
                anyApplied = true;
            }

            // Cubes (main + extras)
            if (!gSelCubes.empty()) {
                for (int ci : gSelCubes) {
                    if (ci == 0) {
                        gCube.pos += delta;
                    } else if (ci > 0) {
                        int idx = ci - 1;
                        if (idx >= 0 && idx < (int)gExtraCubes.size()) {
                            gExtraCubes[idx].pos += delta;
                        }
                    }
                }
                anyApplied = true;
            }

            // Viewport Anchors
            if (!gSelAnchors.empty()) {
                for (int ai : gSelAnchors) {
                    if (ai >= 0 && ai < (int)gAnchors.size()) {
                        gAnchors[ai].pos += delta;
                    }
                }
                anyApplied = true;
            } else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
                gAnchors[gSelectedAnchor].pos += delta;
                anyApplied = true;
            }

            // Cameras (position only on plane path)
            if (!gSelCams.empty()) {
                for (int ci : gSelCams) {
                    if (ci >= 0 && ci < (int)gSceneCams.size()) {
                        gSceneCams[ci].pos += delta;
                    }
                }
                anyApplied = true;
            } else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
                gSceneCams[gSelectedCam].pos += delta;
                anyApplied = true;
            }

            // Fallback to single current target (in case no list matched)
            if (!anyApplied) {
                if (glm::vec3* Ppos = currentPosPtr())
                    *Ppos = *Ppos + delta;
            }

            // Advance the grab point under cursor for stable incremental deltas
            gMove.dragGrabW = hit;

            // UI deltas shown in the frozen local frame
            const glm::vec3 localDelta = glm::transpose(B0) * delta;
            gMove.ui_dx += localDelta.x * m2u;
            gMove.ui_dy += localDelta.y * m2u;
            gMove.ui_dz += localDelta.z * m2u;
        }
    }

    else if (gMove.hot == MoveState::ArcX || gMove.hot == MoveState::ArcY || gMove.hot == MoveState::ArcZ) {
        const glm::vec3 ax = gMove.dragStartAxis;        // frozen at mouse-down
        glm::vec3 hit;
        if (rayPlane(ro, rd, pw0, ax, hit)) {
            glm::vec3 v0 = glm::normalize(gMove.dragGrabW - pw0);
            glm::vec3 v1 = glm::normalize(hit          - pw0);

            // OLD BUILD FORMULA (fixes direction)
            float ang = std::atan2(glm::dot(glm::cross(v0, v1), ax), glm::dot(v0, v1));

            // optional snapping (same as you have)
            const bool snap5 = (glfwGetKey(Win, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS) || (glfwGetKey(Win, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
            const bool snap1 = (glfwGetKey(Win, GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS) || (glfwGetKey(Win, GLFW_KEY_RIGHT_CONTROL)==GLFW_PRESS);
            const float inc  = snap1 ? glm::radians(1.f) : (snap5 ? glm::radians(5.f) : 0.f);
            if (inc > 0.f) ang = std::round(ang / inc) * inc;

            const glm::quat q = glm::angleAxis(ang, glm::normalize(ax));

            bool anyApplied = false;

            // Meshes
            if (!gSelectedMeshes.empty()) {
                for (int mi : gSelectedMeshes) {
                    if (mi < 0 || mi >= (int)gMeshes.size()) continue;
                    glm::vec3 rel = gMeshes[mi].pos - pw0; rel = q * rel; gMeshes[mi].pos = pw0 + rel;
                    gMeshes[mi].rot = glm::normalize(q * gMeshes[mi].rot);
                }
                anyApplied = true;
            }

            // Cubes
            if (!gSelCubes.empty()) {
                for (int ci : gSelCubes) {
                    if (ci == 0) {
                        glm::vec3 rel = gCube.pos - pw0; rel = q * rel; gCube.pos = pw0 + rel;
                        gCube.rot = glm::normalize(q * gCube.rot);
                    } else if (ci > 0) {
                        int idx = ci - 1;
                        if (idx >= 0 && idx < (int)gExtraCubes.size()) {
                            glm::vec3 rel = gExtraCubes[idx].pos - pw0; rel = q * rel; gExtraCubes[idx].pos = pw0 + rel;
                            gExtraCubes[idx].rot = glm::normalize(q * gExtraCubes[idx].rot);
                        }
                    }
                }
                anyApplied = true;
            }

            // Anchors (translate about pivot path)
            if (!gSelAnchors.empty()) {
                for (int ai : gSelAnchors) {
                    if (ai >= 0 && ai < (int)gAnchors.size()) {
                        glm::vec3 rel = gAnchors[ai].pos - pw0; rel = q * rel; gAnchors[ai].pos = pw0 + rel;
                    }
                }
                anyApplied = true;
            } else if (gSelectedAnchor >= 0 && gSelectedAnchor < (int)gAnchors.size()) {
                glm::vec3 rel = gAnchors[gSelectedAnchor].pos - pw0; rel = q * rel; gAnchors[gSelectedAnchor].pos = pw0 + rel;
                anyApplied = true;
            }

            // Cameras (pos + rot)
            if (!gSelCams.empty()) {
                for (int ci : gSelCams) {
                    if (ci >= 0 && ci < (int)gSceneCams.size()) {
                        glm::vec3 rel = gSceneCams[ci].pos - pw0; rel = q * rel; gSceneCams[ci].pos = pw0 + rel;
                        gSceneCams[ci].rot = glm::normalize(q * gSceneCams[ci].rot);
                    }
                }
                anyApplied = true;
            } else if (gSelectedCam >= 0 && gSelectedCam < (int)gSceneCams.size()) {
                glm::vec3 rel = gSceneCams[gSelectedCam].pos - pw0; rel = q * rel; gSceneCams[gSelectedCam].pos = pw0 + rel;
                gSceneCams[gSelectedCam].rot = glm::normalize(q * gSceneCams[gSelectedCam].rot);
                anyApplied = true;
            }

            // Fallback single target
            if (!anyApplied) {
                if (glm::vec3* Ppos = currentPosPtr()) {
                    glm::vec3 rel = *Ppos - pw0; rel = q * rel; *Ppos = pw0 + rel;
                }
                if (glm::quat* Prot = currentRotPtr()) *Prot = glm::normalize(q * (*Prot));
            }

            // advance arc reference vector (old build behavior)
            gMove.dragGrabW = hit;

            // UI meters
            const float deg = glm::degrees(ang);
            if (gMove.hot==MoveState::ArcX) gMove.ui_ax += deg;
            if (gMove.hot==MoveState::ArcY) gMove.ui_ay += deg;
            if (gMove.hot==MoveState::ArcZ) gMove.ui_az += deg;
        }
    }

}

}

//---------------------------------- Anchor Picker ------------------------------
static int pickAnchorAt(double mx, double my, float* outT = nullptr)
{
    glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
    glm::mat4 V = Cam.view();
    glm::mat4 invPV = glm::inverse(P * V);
    glm::vec3 ro = glm::vec3(glm::inverse(V)[3]);
    glm::vec3 rd = screenToRay(float(mx), float(my), invPV); // normalized

    float Ranc = 0.02f * std::max({ S.width_m, S.length_m, S.depth_m });
    Ranc = std::clamp(Ranc, 0.08f, 1.5f);

    int best = -1; float bestT = 1e9f;
    for (int i = 0; i < (int)gAnchors.size(); ++i) {
        if (!gAnchors[i].visible) continue;
        glm::vec3 oc = ro - gAnchors[i].pos;
        float b = glm::dot(oc, rd);
        float c = glm::dot(oc, oc) - Ranc*Ranc;
        float disc = b*b - c;
        if (disc < 0.f) continue;
        float t = -b - std::sqrt(disc);
        if (t > 0.f && t < bestT) { bestT = t; best = i; }
    }
    if (outT) *outT = bestT;
    return best;
}

/* mouse button callback -----------------------------------------------------*/

// Robust modifier reader (GLFW mods + ImGui + direct key query)
static inline void getShiftCtrl(bool& shift, bool& ctrl, int mods, GLFWwindow* win)
{
    ImGuiIO& io = ImGui::GetIO();
    shift = ((mods & GLFW_MOD_SHIFT)   != 0) || io.KeyShift
          || (glfwGetKey(win ? win : Win, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS)
          || (glfwGetKey(win ? win : Win, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    ctrl  = ((mods & GLFW_MOD_CONTROL) != 0) || io.KeyCtrl
          || (glfwGetKey(win ? win : Win, GLFW_KEY_LEFT_CONTROL)  == GLFW_PRESS)
          || (glfwGetKey(win ? win : Win, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);
}


static void mouseBtnCB(GLFWwindow* w, int button, int action, int mods) {
    double mx, my; glfwGetCursorPos(w, &mx, &my);

    // We allow clicks on the nav cube & Home button even if ImGui wants the mouse.
    // Everywhere else, we respect ImGui capture.
    auto ioWantsMouse = []{
        ImGuiIO& io = ImGui::GetIO();
        return io.WantCaptureMouse;
    };

    // --- LMB PRESS -----------------------------------------------------------
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        // --- Home button: fit view ---
        if (inHomeBtnRect()) { frameAll(); return; }

        // 2) Nav cube: start a PENDING click (decide later if it's a snap or a drag)
        if (inCubeRect(mx, my)) {
            cubePendingClick = true;
            cubePressX = mx; cubePressY = my;
            cubePressFace = pickCubeFaceAt(mx, my);   // may be -1 if between edges
            // prime drag refs
            cubeDragLastX = mx; cubeDragLastY = my;
            return;
        }

        // Gizmo does NOT get priority on press; selection runs first.
        if (gMove.active && gMove.hot != MoveState::None) {
            // intentionally do nothing here; drag will begin after picking
        }
        // --- Viewport Anchor picking (only when Move Object = ViewportAnchors) ---
        if (!ioWantsMouse() && gMove.obj == MoveObj::ViewportAnchors) {
            bool shift=false, ctrl=false;
            getShiftCtrl(shift, ctrl, mods, w);

            // Build ray in WORLD
            glm::mat4 P  = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
            glm::mat4 V  = Cam.view();
            glm::mat4 invPV = glm::inverse(P * V);
            double mx2, my2; glfwGetCursorPos(Win, &mx2, &my2);
            glm::vec3 roW = glm::vec3(glm::inverse(V)[3]);
            glm::vec3 rdW = screenToRay((float)mx2, (float)my2, invPV);

            PickHit hit = pickFacesOrPoints(roW, rdW);
            if (hit.kind == PickHit::ANCHOR) {
                const int ai = hit.idx;

                auto inList = [&](int v){ return std::find(gSelAnchors.begin(), gSelAnchors.end(), v) != gSelAnchors.end(); };
                auto add    = [&](int v){ if (!inList(v)) gSelAnchors.push_back(v); };
                auto remove = [&](int v){
                    auto it = std::find(gSelAnchors.begin(), gSelAnchors.end(), v);
                    if (it != gSelAnchors.end()) gSelAnchors.erase(it);
                };

                if (ctrl) {
                    if (inList(ai)) {
                        bool wasPrimary = (gSelectedAnchor == ai);
                        remove(ai);
                        if (wasPrimary)
                            gSelectedAnchor = gSelAnchors.empty() ? -1 : gSelAnchors.front();
                    } else {
                        add(ai);
                        if (gSelectedAnchor < 0) gSelectedAnchor = ai;
                    }
                } else if (shift) {
                    add(ai);
                    if (gSelectedAnchor < 0) gSelectedAnchor = ai;
                } else {
                    // Single select
                    gSelAnchors.clear();
                    gSelAnchors.push_back(ai);
                    gSelectedAnchor = ai;

                    // Clear unrelated selections on plain click
                    gSelectedCam    = -1; gSelCams.clear();
                    gSelectedMesh   = -1; gSelectedMeshes.clear();
                    gCube.selected  = false; gSelectedCube = -1;
                }

                if (gMove.active) refreshMoveSnapshotForSelection();
                gMove.pivot.localPos   = glm::vec3(0);
                gMove.pivot.localBasis = glm::mat3(1.0f);
                return; // don't fall through
            }
            // miss: allow other handlers to try
        }

        // --- Scene Camera picking (only when Move Object = Cameras) ---
        if (!ioWantsMouse() && gMove.obj == MoveObj::Cameras) {
            bool shift=false, ctrl=false;
            getShiftCtrl(shift, ctrl, mods, w);

            // Build ray in WORLD
            glm::mat4 P  = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
            glm::mat4 V  = Cam.view();
            glm::mat4 invPV = glm::inverse(P * V);
            double mx3, my3; glfwGetCursorPos(Win, &mx3, &my3);
            glm::vec3 roW = glm::vec3(glm::inverse(V)[3]);
            glm::vec3 rdW = screenToRay((float)mx3, (float)my3, invPV);

            PickHit hit = pickFacesOrPoints(roW, rdW);
            if (hit.kind == PickHit::CAMERA) {
                const int ci = hit.idx;

                auto inList = [&](int v){ return std::find(gSelCams.begin(), gSelCams.end(), v) != gSelCams.end(); };
                auto add    = [&](int v){ if (!inList(v)) gSelCams.push_back(v); };
                auto remove = [&](int v){
                    auto it = std::find(gSelCams.begin(), gSelCams.end(), v);
                    if (it != gSelCams.end()) gSelCams.erase(it);
                };

                if (ctrl) {
                    if (inList(ci)) {
                        bool wasPrimary = (gSelectedCam == ci);
                        remove(ci);
                        if (wasPrimary)
                            gSelectedCam = gSelCams.empty() ? -1 : gSelCams.front();
                    } else {
                        add(ci);
                        if (gSelectedCam < 0) gSelectedCam = ci;
                    }
                } else if (shift) {
                    add(ci);
                    if (gSelectedCam < 0) gSelectedCam = ci;
                } else {
                    // Single select
                    gSelCams.clear();
                    gSelCams.push_back(ci);
                    gSelectedCam = ci;

                    // Clear unrelated types on plain click
                    gSelectedAnchor = -1; gSelAnchors.clear();
                    gSelectedMesh   = -1; gSelectedMeshes.clear();
                    gCube.selected  = false; gSelectedCube = -1;
                }

                // Ensure Move targets cameras & snapshot baselines
                gMove.obj = MoveObj::Cameras;
                if (gMove.active) refreshMoveSnapshotForSelection();

                // World-aligned pivot at camera origin for gizmo
                gMove.pivot.localPos   = glm::vec3(0);
                gMove.pivot.localBasis = glm::mat3(1.0f);

                return; // handled LMB press
            }
            // miss: allow other handlers to try
        }


        // --- Body selection with Shift/Ctrl (multi-select) ---
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !ioWantsMouse()) {
            // More robust modifier detection: GLFW mods OR ImGui live state OR GLFW key query
            bool shift=false, ctrl=false;
            getShiftCtrl(shift, ctrl, mods, w);   // <- use the window from this callback

            // Ray from mouse
            double mx,my; glfwGetCursorPos(Win,&mx,&my);
            glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
            glm::mat4 V = Cam.view();
            glm::mat4 invPV = glm::inverse(P * V);
            glm::vec3 roW = glm::vec3(glm::inverse(V)[3]);
            glm::vec3 rdW = screenToRay(float(mx), float(my), invPV);
            // Give the move gizmo priority if we're already in a Move session.
            if (gMove.active && gMove.hot != MoveState::None) {
                gMove.dragging    = true;
                glfwSwapInterval(0); // optional: reduce latency during drag
                gMove.prevMouseX  = (float)mx;
                gMove.prevMouseY  = (float)my;

                // Use the ray we just built and freeze the drag frame now
                glm::vec3 ro = roW;
                glm::vec3 rd = rdW;
                glm::vec3 pw = currentPivotWorldPos();
                glm::mat3 B  = currentPivotWorldBasis();

                if (glm::vec3* P = currentPosPtr()) {
                    gMove.dragBasePos = *P;
                } else {
                    gMove.dragging = false;
                    glfwSwapInterval(1);
                    return;
                }

                if (glm::quat* R = currentRotPtr()) {
                    gMove.dragBaseRot = *R;
                } else {
                    gMove.dragBaseRot = glm::quat(1,0,0,0); // identity for non-rotating targets (anchors)
                }

                gMove.dragStartBasis  = B;   // freeze basis at mouse-down
                gMove.dragPivotStart  = pw;  // freeze pivot at mouse-down

                // --- Snapshot baselines for ALL currently selected of the active Move object ---
                gMove.multiSnapPos.clear();          // meshes
                gMove.multiSnapCubePos.clear();      // cubes
                gMove.multiSnapAnchorPos.clear();    // anchors (NEW)
                gMove.multiSnapCamPos.clear();       // cameras pos (NEW)
                gMove.multiSnapCamRot.clear();       // cameras rot (NEW)

                // Meshes
                for (int mi : gSelectedMeshes) {
                    if (mi >= 0 && mi < (int)gMeshes.size()) {
                        gMove.multiSnapPos[mi] = gMeshes[mi].pos;
                    }
                }

                // Cubes (0 = main cube, 1..N = extras by selection id)
                for (int selId : gSelCubes) {
                    if (selId == 0) {
                        gMove.multiSnapCubePos[0] = gCube.pos;
                    } else if (selId > 0) {
                        int idx = selId - 1;
                        if (idx >= 0 && idx < (int)gExtraCubes.size())
                            gMove.multiSnapCubePos[selId] = gExtraCubes[idx].pos;
                    }
                }

                // Anchors (NEW)
                for (int ai : gSelAnchors) {
                    if (ai >= 0 && ai < (int)gAnchors.size())
                        gMove.multiSnapAnchorPos[ai] = gAnchors[ai].pos;
                }

                // Cameras (NEW)
                for (int ci : gSelCams) {
                    if (ci >= 0 && ci < (int)gSceneCams.size()) {
                        gMove.multiSnapCamPos[ci] = gSceneCams[ci].pos;
                        gMove.multiSnapCamRot[ci] = gSceneCams[ci].rot;
                    }
                }


                if (gMove.hot == MoveState::AxisX || gMove.hot == MoveState::AxisY || gMove.hot == MoveState::AxisZ) {
                    // World axis (unit) frozen for this drag
                    glm::vec3 ax = (gMove.hot==MoveState::AxisX)?B[0]:(gMove.hot==MoveState::AxisY?B[1]:B[2]);
                    ax = glm::normalize(ax);

                    // Constraint plane normal: n = cross(ax, cross(rd, ax)), with fallback
                    glm::vec3 n = glm::cross(ax, glm::cross(rd, ax));
                    if (glm::length(n) < 1e-4f) {
                        n = (std::abs(glm::dot(ax, glm::vec3(0,1,0))) < 0.9f)
                              ? glm::cross(ax, glm::vec3(0,1,0))
                              : glm::cross(ax, glm::vec3(1,0,0));
                    }
                    n = glm::normalize(n);

                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, n, hit)) {
                        gMove.dragStartAxis    = ax;
                        gMove.dragStartPw      = pw;
                        gMove.dragStartPlaneN  = n;
                        gMove.dragStart_tAxis  = glm::dot(hit - pw, ax); // project plane-hit onto axis
                        gMove.dragGrabW        = hit;
                    } else {
                        gMove.dragging = false;
                        glfwSwapInterval(1);
                    }
                }
                else if (gMove.hot == MoveState::PlaneXY || gMove.hot == MoveState::PlaneXZ || gMove.hot == MoveState::PlaneYZ) {
                    glm::vec3 n = (gMove.hot==MoveState::PlaneXY)?B[2]:(gMove.hot==MoveState::PlaneXZ?B[1]:B[0]);
                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, glm::normalize(n), hit)) {
                        gMove.dragGrabW        = hit;
                        gMove.dragStartPlaneN  = glm::normalize(n);
                    } else {
                        gMove.dragging = false;
                        glfwSwapInterval(1);
                    }
                }
                else if (gMove.hot == MoveState::ArcX || gMove.hot == MoveState::ArcY || gMove.hot == MoveState::ArcZ) {
                    glm::vec3 ax = (gMove.hot==MoveState::ArcX)?B[0]:(gMove.hot==MoveState::ArcY?B[1]:B[2]);
                    ax = glm::normalize(ax);
                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, ax, hit)) {
                        gMove.dragGrabW      = hit;
                        gMove.dragStartAngle = 0.f;
                        gMove.dragStartAxis  = ax;
                        gMove.dragStartPw    = pw;
                    } else {
                        gMove.dragging = false;
                        glfwSwapInterval(1);
                    }
                }

                return; // consume: do not run body-picking when starting a gizmo drag
            }
            gPickPreferUnselected = (shift || ctrl);

            PickHit hit = pickFacesOrPoints(roW, rdW); // we only care which body was hit
            gPickPreferUnselected = false;

            auto addMeshSel = [&](int mi){
                if (mi < 0 || mi >= (int)gMeshes.size()) return;
                if (std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), mi) == gSelectedMeshes.end())
                    gSelectedMeshes.push_back(mi);
                if (gMove.active) { // snapshot if added mid-move
                    if (!gMove.multiSnapPos.count(mi)) gMove.multiSnapPos[mi] = gMeshes[mi].pos;
                }
                if (gMove.primaryMeshForGizmo < 0) gMove.primaryMeshForGizmo = mi; // first one anchors gizmo
            };
            auto removeMeshSel = [&](int mi){
                auto it = std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), mi);
                if (it != gSelectedMeshes.end()) {
                    // if deselect during move, restore that body's original pose
                    if (gMove.active) {
                        auto itSnap = gMove.multiSnapPos.find(mi);
                        if (itSnap != gMove.multiSnapPos.end()) gMeshes[mi].pos = itSnap->second;
                    }
                    gSelectedMeshes.erase(it);
                    if (gSelectedMeshes.empty()) gMove.primaryMeshForGizmo = -1;
                    else if (gMove.primaryMeshForGizmo == mi) gMove.primaryMeshForGizmo = gSelectedMeshes.front();
                }
            };

            if (hit.kind == PickHit::MESH) {
                int mi = hit.idx;

                if (ctrl) {
                    // toggle
                    if (std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), mi) != gSelectedMeshes.end())
                        removeMeshSel(mi);
                    else
                        addMeshSel(mi);
                } else if (shift) {
                    // add
                    addMeshSel(mi);
                } else {
                    // single select
                    gSelectedMeshes.clear();
                    gSelectedMeshes.push_back(mi);
                    gMove.primaryMeshForGizmo = mi;
                    if (!gMove.dragging) {
                        gSelectedMesh = mi;
                        if (gMove.active) refreshMoveSnapshotForSelection();
                    }
                    // clear cubes
                    gSelectedCube = -1;
                    gCube.selected = false;
                }
                // done
                return;
            }
            else if (hit.kind == PickHit::CUBE) {
                // Clear mesh selections (meshes and cubes are exclusive for Bodies move)
                gSelectedMeshes.clear();
                gMove.primaryMeshForGizmo = -1;
                gSelectedMesh = -1;

                const int ci = hit.idx; // 0 = main cube, >0 extras

                auto addCube = [&](int idx){
                    if (std::find(gSelCubes.begin(), gSelCubes.end(), idx) == gSelCubes.end())
                        gSelCubes.push_back(idx);
                    if (gSelectedCube < 0)
                        gSelectedCube = idx; // first selected becomes primary for gizmo
                };
                auto removeCube = [&](int idx){
                    auto it = std::find(gSelCubes.begin(), gSelCubes.end(), idx);
                    if (it != gSelCubes.end()) gSelCubes.erase(it);
                    if (gSelectedCube == idx)
                        gSelectedCube = gSelCubes.empty() ? -1 : gSelCubes.front();
                };

                if (ctrl) {
                    // toggle membership
                    if (std::find(gSelCubes.begin(), gSelCubes.end(), ci) != gSelCubes.end())
                        removeCube(ci);
                    else
                        addCube(ci);
                } else if (shift) {
                    // add
                    addCube(ci);
                } else {
                    // single select
                    gSelCubes.clear();
                    gSelCubes.push_back(ci);
                    gSelectedCube = ci;
                }

                gCube.selected = (gSelectedCube == 0);

                if (gMove.active) refreshMoveSnapshotForSelection();

                // sensible world-aligned pivot for Bodies
                gMove.pivot.localPos   = glm::vec3(0);
                gMove.pivot.localBasis = glm::mat3(1.0f);
                return;
            }

            // If we got here, either the hit wasn't a mesh/cube or we chose not to consume it.
            // Clicking empty space (no shift/ctrl) will clear selection when not moving.
            else if (hit.kind == PickHit::NONE) {
                if (!gMove.active && !shift && !ctrl) {
                    gSelectedCube = -1; gCube.selected = false;
                    gSelCubes.clear();
                    gSelectedMesh = -1; gSelectedMeshes.clear();
                    gSelectedCam  = -1; gSelCams.clear();
                    gSelectedAnchor = -1; gSelAnchors.clear();
                    gMove.primaryMeshForGizmo = -1;
                }
                return;
            }

            return;


        }

        // --- Face/Point picking (precise) ---
        if (!ioWantsMouse() && !gMove.pickLocked && (gMove.obj == MoveObj::Faces || gMove.obj == MoveObj::Points)) {    // Build cursor ray in WORLD
            glm::mat4 P  = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
            glm::mat4 V  = Cam.view();
            glm::mat4 invPV = glm::inverse(P * V);
            glm::vec3 roW = glm::vec3(glm::inverse(V)[3]);
            glm::vec3 rdW = screenToRay(float(mx), float(my), invPV);

            PickHit hit = pickFacesOrPoints(roW, rdW);

            if (hit.kind == PickHit::CUBE) {
                // Clear unrelated selections
                gSelectedAnchor = -1;
                gSelectedCam    = -1;
                gSelectedMesh   = -1;

                gSelectedCube = hit.idx;                  // 0 = main cube, >0 extras
                gCube.selected = (gSelectedCube == 0);
                if (gMove.active) refreshMoveSnapshotForSelection();


                // --- Cube: POINTS = nearest vertex; FACES = face center ---
                if (gMove.obj == MoveObj::Points) {
                    const glm::vec3 v = hit.pLocal;
                    glm::vec3 snapped(
                        (v.x >= 0.f ? 0.5f : -0.5f),
                        (v.y >= 0.f ? 0.5f : -0.5f),
                        (v.z >= 0.f ? 0.5f : -0.5f)
                    );
                    gMove.pivot.localPos   = snapped;
                    gMove.pivot.localBasis = glm::mat3(1.0f);
                } else { // Faces
                    glm::vec3 absP = glm::abs(hit.pLocal);
                    glm::vec3 nLocal(0.f), center(0.f);
                    if (absP.x > absP.y && absP.x > absP.z) {
                        nLocal = glm::vec3((hit.pLocal.x > 0.f) ? 1.f : -1.f, 0.f, 0.f);
                        center = glm::vec3((hit.pLocal.x > 0.f) ? 0.5f : -0.5f, 0.f, 0.f);
                    } else if (absP.y > absP.x && absP.y > absP.z) {
                        nLocal = glm::vec3(0.f, (hit.pLocal.y > 0.f) ? 1.f : -1.f, 0.f);
                        center = glm::vec3(0.f, (hit.pLocal.y > 0.f) ? 0.5f : -0.5f, 0.f);
                    } else {
                        nLocal = glm::vec3(0.f, 0.f, (hit.pLocal.z > 0.f) ? 1.f : -1.f);
                        center = glm::vec3(0.f, 0.f, (hit.pLocal.z > 0.f) ? 0.5f : -0.5f);
                    }
                    gMove.pivot.localPos   = center;
                    gMove.pivot.localBasis = faceFrame(nLocal);
                }

                gMove.pickLocked = true; // lock until cleared
            }
            else if (hit.kind == PickHit::MESH) {
                // Clear unrelated selections
                gSelectedAnchor = -1;
                gSelectedCam    = -1;
                gCube.selected  = false;
                gSelectedCube   = -1;

                gSelectedMesh = hit.idx;
                if (gMove.active) refreshMoveSnapshotForSelection();

                // Mesh data
                const int mi = hit.idx;
                if (mi >= 0 && mi < (int)gMeshes.size() && hit.triStart >= 0) {
                    const ImportedMesh& Msh = gMeshes[mi];

                    // triStart is an index into cpuIndices
                    const int k0 = hit.triStart + 0;
                    const int k1 = hit.triStart + 1;
                    const int k2 = hit.triStart + 2;
                    if (k2 < (int)Msh.cpuIndices.size()) {
                        const uint32_t i0 = Msh.cpuIndices[k0];
                        const uint32_t i1 = Msh.cpuIndices[k1];
                        const uint32_t i2 = Msh.cpuIndices[k2];
                        if (i0 < Msh.cpuVerts.size() && i1 < Msh.cpuVerts.size() && i2 < Msh.cpuVerts.size()) {
                            const glm::vec3& a = Msh.cpuVerts[i0];
                            const glm::vec3& b = Msh.cpuVerts[i1];
                            const glm::vec3& c = Msh.cpuVerts[i2];

                            if (gMove.obj == MoveObj::Points) {
                                // nearest tri vertex (local space)
                                const float da = glm::dot(hit.pLocal - a, hit.pLocal - a);
                                const float db = glm::dot(hit.pLocal - b, hit.pLocal - b);
                                const float dc = glm::dot(hit.pLocal - c, hit.pLocal - c);
                                const glm::vec3 nearest = (da < db) ? ((da < dc) ? a : c) : ((db < dc) ? b : c);

                                gMove.pivot.localPos   = nearest;
                                gMove.pivot.localBasis = glm::mat3(1.0f);
                            } else { // Faces = triangle centroid
                                const glm::vec3 centroid = (a + b + c) / 3.0f;
                                gMove.pivot.localPos   = centroid;
                                gMove.pivot.localBasis = faceFrame(hit.nLocal);
                            }

                            gMove.pickLocked = true; // lock until cleared
                        }
                    }
                }
            } else {
                // click miss: only clear on actual empty space (no hit) and NOT actively moving
                if (!gMove.active && hit.kind == PickHit::NONE) {
                    gSelectedAnchor = -1; gSelAnchors.clear();
                    gSelectedCam    = -1; gSelCams.clear();
                    gSelectedMeshes.clear(); gSelectedMesh = -1;
                    gCube.selected  = false; gSelectedCube = -1;
                }
            }
            // 4) Move gizmo: begin drag if a handle is hot
            if (gMove.active && gMove.hot != MoveState::None) {
                gMove.dragging = true;
                glfwSwapInterval(0); // disable V-Sync during drag for lower input latency (optional)
                gMove.prevMouseX = (float)mx;
                gMove.prevMouseY = (float)my;

                // Prepare baseline for axis/plane/arc drags
                glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
                glm::mat4 V = Cam.view();
                glm::mat4 invPV = glm::inverse(P * V);
                glm::vec3 ro = glm::vec3(glm::inverse(V)[3]);
                glm::vec3 rd = screenToRay(float(mx), float(my), invPV);

                glm::vec3 pw = currentPivotWorldPos();
                glm::mat3 B  = currentPivotWorldBasis();

                if (glm::vec3* P = currentPosPtr()) {
                    gMove.dragBasePos = *P;
                } else {
                    gMove.dragging = false;
                    glfwSwapInterval(1);
                    return;
                }
                if (glm::quat* R = currentRotPtr()) {
                    gMove.dragBaseRot = *R;
                } else {
                    gMove.dragBaseRot = glm::quat(1,0,0,0);
                }
                gMove.dragStartBasis = B;   // freeze basis at mouse-down
                gMove.dragPivotStart = pw;  // freeze pivot at mouse-down



                if (gMove.hot == MoveState::AxisX || gMove.hot == MoveState::AxisY || gMove.hot == MoveState::AxisZ) {
                    // Axis direction in world space (unit), frozen for the drag session
                    glm::vec3 ax = (gMove.hot==MoveState::AxisX)?B[0]:(gMove.hot==MoveState::AxisY?B[1]:B[2]);
                    ax = glm::normalize(ax);

                    // Camera-aware constraint plane: normal is the camera ray component orthogonal to the axis.
                    // n = cross(ax, cross(rd, ax)) ; robust fallback if nearly parallel.
                    glm::vec3 n = glm::cross(ax, glm::cross(rd, ax));
                    const float nl = glm::length(n);
                    if (nl < 1e-4f) {
                        n = (std::abs(glm::dot(ax, glm::vec3(0,1,0))) < 0.9f)
                              ? glm::cross(ax, glm::vec3(0,1,0))
                              : glm::cross(ax, glm::vec3(1,0,0));
                    }
                    n = glm::normalize(n);

                    // Intersect current mouse ray with the plane through pivot with normal n
                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, n, hit)) {
                        // IMPORTANT: baseline from plane-hit projected onto axis, not line–line closest
                        const float tStart = glm::dot(hit - pw, ax);

                        gMove.dragStartAxis   = ax;
                        gMove.dragStartPw     = pw;
                        gMove.dragStartPlaneN = n;
                        gMove.dragStart_tAxis = tStart;
                        gMove.dragGrabW       = hit;  // the visible grabbed point
                    } else {
                        gMove.dragging = false;       // degenerate, abort
                    }
                }

                else if (gMove.hot == MoveState::PlaneXY || gMove.hot == MoveState::PlaneXZ || gMove.hot == MoveState::PlaneYZ) {
                    glm::vec3 n = (gMove.hot==MoveState::PlaneXY)?B[2]:(gMove.hot==MoveState::PlaneXZ?B[1]:B[0]);
                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, glm::normalize(n), hit)) {
                        gMove.dragGrabW     = hit;
                        gMove.dragStartPlaneN = glm::normalize(n);
                    } else {
                        gMove.dragging = false;
                        glfwSwapInterval(1); // restore V-Sync after drag (optional)

                    }
                }
                else if (gMove.hot == MoveState::ArcX || gMove.hot == MoveState::ArcY || gMove.hot == MoveState::ArcZ) {
                    glm::vec3 ax = (gMove.hot==MoveState::ArcX)?B[0]:(gMove.hot==MoveState::ArcY?B[1]:B[2]);
                    ax = glm::normalize(ax);
                    glm::vec3 hit;
                    if (rayPlane(ro, rd, pw, ax, hit)) {
                        gMove.dragGrabW      = hit;
                        gMove.dragStartAngle = 0.f;
                        gMove.dragStartAxis  = ax;   // <-- store chosen arc axis (fix)
                        gMove.dragStartPw    = pw;   // <-- keep pivot for consistency
                    } else {
                        gMove.dragging = false;
                        glfwSwapInterval(1); // restore V-Sync after drag (optional)

                    }
                }

                return; // consume
            }
            // 5) Scene picking (when not starting a gizmo drag)
            // Build a world-space ray from the click and pick cube(s)/meshes.
            // Clicking empty space will deselect (if not in the middle of a move).
            {
                // Build world ray
                glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
                glm::mat4 V = Cam.view();
                glm::mat4 invPV = glm::inverse(P * V);
                glm::vec3 ro = glm::vec3(glm::inverse(V)[3]);
                glm::vec3 rd = screenToRay(float(mx), float(my), invPV);

                auto hitBoxLocal = [&](const glm::vec3& bmin, const glm::vec3& bmax,
                                       const glm::mat4& M, float& outT)->bool
                {
                    // Transform ray to local
                    glm::mat4 invM = glm::inverse(M);
                    glm::vec3 rO = glm::vec3(invM * glm::vec4(ro,1));
                    glm::vec3 rD = glm::normalize(glm::vec3(invM * glm::vec4(rd,0)));

                    glm::vec3 tmin = (bmin - rO) / rD;
                    glm::vec3 tmax = (bmax - rO) / rD;
                    glm::vec3 t0   = glm::min(tmin, tmax);
                    glm::vec3 t1   = glm::max(tmin, tmax);
                    float tEnter = std::max({t0.x, t0.y, t0.z});
                    float tExit  = std::min({t1.x, t1.y, t1.z});
                    if (tEnter <= tExit && tExit > 0.f) { outT = tEnter; return true; }
                    return false;
                };

                enum class Hit { NONE, CUBE, MESH };
                struct Best { Hit kind=Hit::NONE; int idx=-1; float t=1e9f; } best;

                // main cube
                {
                    glm::mat4 M = glm::translate(glm::mat4(1.f), gCube.pos)
                                * glm::mat4_cast(gCube.rot)
                                * glm::scale (glm::mat4(1.f), glm::vec3(gCube.scale));
                    float t;
                    if (hitBoxLocal(glm::vec3(-0.5f), glm::vec3(+0.5f), M, t) && t < best.t) {
                        best = { Hit::CUBE, 0, t };
                    }
                }
                // extra cubes
                for (int i=0;i<(int)gExtraCubes.size();++i){
                    const auto& C = gExtraCubes[i];
                    glm::mat4 M = glm::translate(glm::mat4(1.f), C.pos)
                                * glm::mat4_cast(C.rot)
                                * glm::scale (glm::mat4(1.f), glm::vec3(C.scale));
                    float t;
                    if (hitBoxLocal(glm::vec3(-0.5f), glm::vec3(+0.5f), M, t) && t < best.t) {
                        best = { Hit::CUBE, i+1, t };
                    }
                }
                // imported meshes (use their local AABB)
                for (int i=0;i<(int)gMeshes.size();++i){
                    const auto& Msh = gMeshes[i];
                    glm::mat4 M = glm::translate(glm::mat4(1.f), Msh.pos)
                                * glm::mat4_cast(Msh.rot)
                                * glm::scale (glm::mat4(1.f), glm::vec3(Msh.scale));
                    float t;
                    if (hitBoxLocal(Msh.aabbMin, Msh.aabbMax, M, t) && t < best.t) {
                        best = { Hit::MESH, i, t };
                    }
                }

                // Apply selection (and clear others)
                if (best.kind == Hit::CUBE) {
                    gSelectedMesh = -1;
                    gSelectedCube = best.idx;
                    gCube.selected = (best.idx == 0);
                    gSelectedCam = -1;
                    gSelectedAnchor = -1;
                } else if (best.kind == Hit::MESH) {
                    gSelectedMesh = best.idx;
                    gSelectedCube = -1;
                    gCube.selected = false;
                    gSelectedCam = -1;
                    gSelectedAnchor = -1;
                } else {
                    if (!gMove.active) {
                        gSelectedMesh = -1;
                        gSelectedCube = -1;
                        gCube.selected = false;
                        gSelectedCam = -1;
                        gSelectedAnchor = -1;
                    }
                }

                return; // handled LMB press
            }

        }

// NEW: If Move is active and a gizmo handle is hot, begin drag immediately and skip any selection logic.


        // Deselect everything by clicking empty space when not moving
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !ioWantsMouse() && !gMove.active) {
            double mx, my; glfwGetCursorPos(Win, &mx, &my);
            glm::mat4 P  = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
            glm::mat4 V  = Cam.view();
            glm::mat4 invPV = glm::inverse(P * V);
            glm::vec3 roW = glm::vec3(glm::inverse(V)[3]);
            glm::vec3 rdW = screenToRay(float(mx), float(my), invPV);

            PickHit hit = pickFacesOrPoints(roW, rdW);
            if (hit.kind == PickHit::NONE) {
                gSelectedCam = -1;        gSelCams.clear();
                gSelectedAnchor = -1;     gSelAnchors.clear();
                gCube.selected = false;   gSelectedCube = -1;
                gSelectedMesh = -1;       gSelectedMeshes.clear();
                gMove.primaryMeshForGizmo = -1;
            }
        }


        // --- LMB RELEASE ---------------------------------------------------------
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE)
        {
            // Finish a cube drag or a pending click
            if (cubeDragging) {
                cubeDragging = false;
                cubePendingClick = false;
                cubePressFace = -1;
                return;
            }
            if (cubePendingClick) {
                // treat as click if tiny motion -> face snap
                const double dx = mx - cubePressX, dy = my - cubePressY;
                const double dist2 = dx*dx + dy*dy;
                if (dist2 < 9.0 && cubePressFace >= 0) {
                    applyFaceSnap(cubePressFace);
                }
                cubePendingClick = false;
                cubePressFace = -1;
                return;
            }

            // If ImGui wants mouse, let it have the release (but still end our drag)
            if (gMove.dragging) {
                // End gizmo drag cleanly
                int lastAxis = -1;
                if (gMove.hot == MoveState::AxisX) lastAxis = 0;
                else if (gMove.hot == MoveState::AxisY) lastAxis = 1;
                else if (gMove.hot == MoveState::AxisZ) lastAxis = 2;

                gMove.dragging = false;
                glfwSwapInterval(1); // restore V-Sync after drag (optional)
                gMove.axisSnapPending = (lastAxis != -1); // enables one-shot face/plane snap
                gMove.lastAxis = lastAxis;
                gMove.hot = MoveState::None;
                // no return; fall through is fine
            }

            if (ioWantsMouse()) return;
        }

        // --- MMB press/release are handled elsewhere; nothing special needed here.
    }
}
static void keyCB(GLFWwindow*, int key, int, int action, int mods) {
    ImGuiIO& io = ImGui::GetIO();

    // Only block shortcuts while actually typing in a text field
    if (io.WantTextInput) return;
    if (action != GLFW_PRESS) return;

    if (key == GLFW_KEY_M) {
        if (action == GLFW_PRESS) {
            if (!gMove.active) {
                // Reset move-session state (always)
                gMove.hot        = MoveState::None;
                gMove.dragging   = false;
                // keep existing pickLocked state only for Points/Faces mode
                gMove.pickLocked = (gMove.obj == MoveObj::Faces || gMove.obj == MoveObj::Points) ? gMove.pickLocked : false;

                // Open the Move panel even with no selection
                gMove.active = true;

                // --- snapshot base for cancel/OK and multi-select support ---
                gMove.multiSnapPos.clear();
                for (int idx : gSelectedMeshes) {
                    if (idx >= 0 && idx < (int)gMeshes.size())
                        gMove.multiSnapPos[idx] = gMeshes[idx].pos;
                }
                if (!gSelectedMeshes.empty()) gMove.primaryMeshForGizmo = gSelectedMeshes.front();

                // Default: no snap target if nothing selected — gizmo stays hidden until you pick
                gMove.snapTarget    = MoveState::SnapTarget::None;
                gMove.snapAnchorIdx = -1;
                gMove.snapCamIdx    = -1;
                gMove.snapCubeSel   = gSelectedCube;

                // If a valid selection exists, initialize snap to it
                if (gMove.obj == MoveObj::ViewportAnchors && gSelectedAnchor >= 0) {
                    gMove.snapTarget    = MoveState::SnapTarget::Anchor;
                    gMove.snapAnchorIdx = gSelectedAnchor;
                    gMove.snapPos = gAnchors[gSelectedAnchor].pos;
                    gMove.snapRot = glm::quat(1,0,0,0);
                }
                else if (gMove.obj == MoveObj::Cameras && gSelectedCam >= 0) {
                    gMove.snapTarget  = MoveState::SnapTarget::Camera;
                    gMove.snapCamIdx  = gSelectedCam;
                    gMove.snapPos = gSceneCams[gSelectedCam].pos;
                    gMove.snapRot = gSceneCams[gSelectedCam].rot;
                }
                else if (gMove.obj == MoveObj::Bodies) {
                    // Prefer meshes if any selected; else cubes
                    int primaryMesh = -1;
                    if (gSelectedMesh >= 0 && gSelectedMesh < (int)gMeshes.size()) {
                        primaryMesh = gSelectedMesh;
                    } else if (!gSelectedMeshes.empty()) {
                        primaryMesh = gSelectedMeshes.front();
                    }

                    if (primaryMesh >= 0 && primaryMesh < (int)gMeshes.size()) {
                        gMove.snapTarget  = MoveState::SnapTarget::Mesh;  // NEW
                        gMove.snapMeshIdx = primaryMesh;                  // NEW
                        gMove.snapPos     = gMeshes[primaryMesh].pos;
                        gMove.snapRot     = gMeshes[primaryMesh].rot;
                    } else if (gCube.selected || gSelectedCube >= 0) {
                        gMove.snapTarget = MoveState::SnapTarget::Cube;
                        const CubeState& C = (gSelectedCube == 0) ? gCube :
                                             (gSelectedCube > 0 && gSelectedCube - 1 < (int)gExtraCubes.size())
                                                 ? gExtraCubes[gSelectedCube - 1] : gCube;
                        gMove.snapPos = C.pos;
                        gMove.snapRot = C.rot;
                    }
                    // else: leave snapTarget=None (no gizmo until user picks something)
                }

                // Faces/Points will remain snapTarget=None here unless you have a pickLock already

                // reset UI deltas / session flags
                gMove.ui_dx = gMove.ui_dy = gMove.ui_dz = 0.f;
                gMove.ui_ax = gMove.ui_ay = gMove.ui_az = 0.f;

                gMove.lastAxis = -1;
                gMove.axisSnapPending = false;

                gMove.cloneChecked   = false;
                gMove.cloneActive    = false;
                gMove.cloneAnchorIdx = -1;
                gMove.cloneCamIdx = -1;
            }
        }
    }

    else if (key == GLFW_KEY_ENTER || key == GLFW_KEY_KP_ENTER) {
        if (gMove.active) {
            // Same as clicking the "OK" button in the Move panel
            if (anchorMode()) {
                gMove.cloneAnchorIdx = -1;
            } else if (cameraMode()) {
                // Camera: if a clone exists it's already in gSceneCams; just clear state
                gMove.cloneCamIdx = -1;
            } else if (gMove.cloneActive) {
                // Commit the temp cube clone
                gExtraCubes.push_back(gMove.cloneCube);
                gMove.cloneActive = false;
                // Select the newly created cube
                gSelectedCube = (int)gExtraCubes.size(); // 1..N -> extras[gSelectedCube-1]
            }

            // End Move session (mirror the OK button cleanup)
            gMove.active = false;
            gMove.multiSnapPos.clear();
            gMove.multiSnapAnchorPos.clear(); // NEW
            gMove.multiSnapCamPos.clear();    // NEW
            gMove.multiSnapCamRot.clear();    // NEW

            gMove.multiSnapCubePos.clear();   // << ADD
            gMove.primaryMeshForGizmo = -1;

            gMove.pickLocked = false;

            gMove.dragging = false;
            glfwSwapInterval(1); // restore V-Sync after drag (optional)

            gMove.hot = MoveState::None;
            gMove.lastAxis = -1;
            gMove.axisSnapPending = false;

            return;
        }
    }

    else if (key == GLFW_KEY_ESCAPE) {
        if (gMove.active) {
            // 1) Drop any temp clones created during this Move session
            if (gMove.obj == MoveObj::ViewportAnchors) {
                if (gMove.cloneAnchorIdx >= 0 && gMove.cloneAnchorIdx < (int)gAnchors.size()) {
                    gAnchors.erase(gAnchors.begin() + gMove.cloneAnchorIdx);
                    gSelectedAnchor = gMove.snapAnchorIdx;
                }
                gMove.cloneAnchorIdx = -1;
            } else if (gMove.obj == MoveObj::Cameras) {
                if (gMove.cloneCamIdx >= 0 && gMove.cloneCamIdx < (int)gSceneCams.size()) {
                    gSceneCams.erase(gSceneCams.begin() + gMove.cloneCamIdx);
                    gSelectedCam = gMove.snapCamIdx;
                }
                gMove.cloneCamIdx = -1;
            } else if (gMove.obj == MoveObj::Bodies) {
                if (gMove.cloneMeshIdx >= 0) {
                    if (gMove.cloneMeshIdx < (int)gMeshes.size())
                        gMeshes.erase(gMeshes.begin() + gMove.cloneMeshIdx);
                    if (gMove.snapTarget == MoveState::SnapTarget::Mesh &&
                        gMove.snapMeshIdx >= 0 && gMove.snapMeshIdx < (int)gMeshes.size())
                    {
                        gSelectedMesh = gMove.snapMeshIdx;
                        gMeshes[gSelectedMesh].pos = gMove.snapPos;
                        gMeshes[gSelectedMesh].rot = gMove.snapRot;
                    } else {
                        gSelectedMesh = -1;
                    }
                    gMove.cloneMeshIdx = -1;
                }
                if (gMove.cloneActive) {
                    gMove.cloneActive = false; // drop temp cube clone
                }
            }

            // 2) Restore the single thing we snapshotted at move start (legacy path)
            switch (gMove.snapTarget) {
                case MoveState::SnapTarget::Anchor:
                    if (gMove.snapAnchorIdx >= 0 && gMove.snapAnchorIdx < (int)gAnchors.size())
                        gAnchors[gMove.snapAnchorIdx].pos = gMove.snapPos;
                    break;

                case MoveState::SnapTarget::Camera:
                    if (gMove.snapCamIdx >= 0 && gMove.snapCamIdx < (int)gSceneCams.size()) {
                        gSceneCams[gMove.snapCamIdx].pos = gMove.snapPos;
                        gSceneCams[gMove.snapCamIdx].rot = gMove.snapRot;
                    }
                    break;

                case MoveState::SnapTarget::Cube: {
                    if (gMove.snapCubeSel == 0) {
                        gCube.pos = gMove.snapPos;
                        gCube.rot = gMove.snapRot;
                    } else if (gMove.snapCubeSel > 0 && gMove.snapCubeSel - 1 < (int)gExtraCubes.size()) {
                        gExtraCubes[gMove.snapCubeSel - 1].pos = gMove.snapPos;
                        gExtraCubes[gMove.snapCubeSel - 1].rot = gMove.snapRot;
                    }
                } break;

                case MoveState::SnapTarget::Mesh:
                    if (gMove.snapMeshIdx >= 0 && gMove.snapMeshIdx < (int)gMeshes.size()) {
                        gMeshes[gMove.snapMeshIdx].pos = gMove.snapPos;
                        gMeshes[gMove.snapMeshIdx].rot = gMove.snapRot;
                    }
                    break;

                case MoveState::SnapTarget::None:
                default: break;
            }

            // 3) Restore ALL items captured in multi-snap maps (true multi-select cancel)
            if (gMove.obj == MoveObj::ViewportAnchors) {
                for (auto& kv : gMove.multiSnapAnchorPos) {
                    int ai = kv.first;
                    if (ai >= 0 && ai < (int)gAnchors.size())
                        gAnchors[ai].pos = kv.second;
                }
            } else if (gMove.obj == MoveObj::Cameras) {
                for (auto& kv : gMove.multiSnapCamPos) {
                    int ci = kv.first;
                    if (ci >= 0 && ci < (int)gSceneCams.size())
                        gSceneCams[ci].pos = kv.second;
                }
                for (auto& kv : gMove.multiSnapCamRot) {
                    int ci = kv.first;
                    if (ci >= 0 && ci < (int)gSceneCams.size())
                        gSceneCams[ci].rot = kv.second;
                }
            } else {
                for (auto& kv : gMove.multiSnapPos) {
                    int mi = kv.first;
                    if (mi >= 0 && mi < (int)gMeshes.size())
                        gMeshes[mi].pos = kv.second;
                }
                for (auto& kv : gMove.multiSnapCubePos) {
                    int sel = kv.first; // 0 = main cube, >0 = extras[sel-1]
                    if (sel == 0) gCube.pos = kv.second;
                    else if (sel > 0 && sel - 1 < (int)gExtraCubes.size())
                        gExtraCubes[sel - 1].pos = kv.second;
                }
            }
            // Clear UI deltas so nothing re-applies on the next frame
            gMove.ui_dx = gMove.ui_dy = gMove.ui_dz = 0.f;
            gMove.ui_ax = gMove.ui_ay = gMove.ui_az = 0.f;

            // 4) End Move session – mirror the OK/Cancel cleanup
            gMove.active = false;
            gMove.multiSnapPos.clear();
            gMove.multiSnapAnchorPos.clear();
            gMove.multiSnapCamPos.clear();
            gMove.multiSnapCamRot.clear();
            gMove.multiSnapCubePos.clear();
            gMove.primaryMeshForGizmo = -1;

            gMove.pickLocked = false;
            gMove.dragging   = false; glfwSwapInterval(1);
            gMove.hot        = MoveState::None;
            gMove.lastAxis   = -1;
            gMove.axisSnapPending = false;

            return;
        }
    }
}
static void scrollCB(GLFWwindow* w, double /*xoff*/, double yoff)
{
    if (ioCaptures()) return;

    // Fallback factor if we can't compute a proper zoom-to-cursor
    const float factor = (yoff < 0) ? 1.10f : 0.90f;

    // Ignore zoom-to-cursor when the mouse is over the orientation cube
    double mx, my; glfwGetCursorPos(Win, &mx, &my);
    const bool overCube = inCubeRect(mx, my);


    auto centerZoom = [&](){
        Cam.dist = std::clamp(Cam.dist * factor, 5.f, 500.f);
    };

    if (overCube) { centerZoom(); return; }

    // --- Build current PV inverse to make a world ray from the mouse ---
    glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
    glm::mat4 V = Cam.view();
    glm::mat4 invPV = glm::inverse(P * V);

    // window -> NDC
    float nx =  2.0f * float(mx) / float(FBW) - 1.0f;
    float ny =  1.0f - 2.0f * float(my) / float(FBH);

    auto unproject = [&](float ndcZ){
        glm::vec4 p = invPV * glm::vec4(nx, ny, ndcZ, 1.0f);
        return glm::vec3(p) / p.w;
    };

    glm::vec3 pNear = unproject(-1.0f);
    glm::vec3 pFar  = unproject( 1.0f);
    glm::vec3 rayO  = pNear;
    glm::vec3 rayD  = glm::normalize(pFar - pNear);

    // Intersect with water plane y = 0
    const float eps = 1e-6f;
    if (std::abs(rayD.y) < eps) { centerZoom(); return; }
    float t0 = -rayO.y / rayD.y;
    if (t0 <= 0.0f) { centerZoom(); return; }
    glm::vec3 hit0 = rayO + t0 * rayD;

    // Apply zoom (change distance) and recompute the hit under the cursor
    float newDist = std::clamp(Cam.dist * factor, 5.f, 500.f);
    float oldDist = Cam.dist;
    Cam.dist = newDist;

    glm::mat4 V2 = Cam.view();
    glm::mat4 invPV2 = glm::inverse(P * V2);
    auto unproject2 = [&](float ndcZ){
        glm::vec4 p = invPV2 * glm::vec4(nx, ny, ndcZ, 1.0f);
        return glm::vec3(p) / p.w;
    };
    glm::vec3 pNear2 = unproject2(-1.0f);
    glm::vec3 pFar2  = unproject2( 1.0f);
    glm::vec3 rayO2  = pNear2;
    glm::vec3 rayD2  = glm::normalize(pFar2 - pNear2);

    if (std::abs(rayD2.y) < eps) { Cam.dist = newDist; return; }
    float t1 = -rayO2.y / rayD2.y;
    if (t1 <= 0.0f) { Cam.dist = newDist; return; }
    glm::vec3 hit1 = rayO2 + t1 * rayD2;

    // Shift the orbit center so the world point under the mouse stays fixed
    Cam.pan += (hit0 - hit1);
}

//---------------------------------- Init & main --------------------------------
static void init(){ if(!glfwInit()) throw std::runtime_error("glfw"); glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,4); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,5); glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    Win=glfwCreateWindow(FBW,FBH,"Water Sim",nullptr,nullptr); if(!Win) throw std::runtime_error("window"); glfwMakeContextCurrent(Win); glfwSwapInterval(1);
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) throw std::runtime_error("glad"); glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glfwSetCursorPosCallback(Win,cursorCB); glfwSetMouseButtonCallback(Win,mouseBtnCB); glfwSetScrollCallback(Win,scrollCB);
    glfwSetKeyCallback(Win, keyCB);
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // ADD THESE TWO LINES:
    ImGui_ImplGlfw_InitForOpenGL(Win, /*install_callbacks=*/true);
    ImGui_ImplOpenGL3_Init("#version 450");

    // then your style / flags as before...
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
#ifdef IMGUI_HAS_VIEWPORT
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;    // OS windows
    // Make sure platform windows use OS chrome (min/max/close) and show in taskbar
    io.ConfigViewportsNoDecoration = false;
    io.ConfigViewportsNoTaskBarIcon = false;

#endif
#ifdef IMGUI_HAS_DOCKING
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;      // (optional, nice UX)
#endif

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
#ifdef IMGUI_HAS_VIEWPORT
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        style.WindowRounding = 6.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }
#endif




    progSurf=link({comp(GL_VERTEX_SHADER,SurfVS), comp(GL_FRAGMENT_SHADER,SurfFS)});
    progCube=link({comp(GL_VERTEX_SHADER,CubeVS), comp(GL_FRAGMENT_SHADER,CubeFS)});
    buildSurface(); buildCube(); buildSphere(); S.waves.resize(1);

    // Apply wireframe state read from config
    glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);

    // Initialize Save/Config folders & load settings.csv
    cfg::initSaveSystem();

    // default pivot at local origin aligned to local axes
    gMove.pivot.localPos = glm::vec3(0,0,0);
    gMove.pivot.localBasis = glm::mat3(1.0f);
}
int main(){ try{ init(); while(!glfwWindowShouldClose(Win)){ glfwPollEvents(); glfwGetFramebufferSize(Win,&FBW,&FBH);
// --- Subview render flags (local to this frame) ---
static bool      gInCamSubview   = false;        // used in Part G
static bool      gUseOverridePV  = false;
static glm::mat4 gOverrideP, gOverrideV;

// Small helper to build a view matrix from a SceneCam
auto viewFromSceneCam = [&](const SceneCam& c)->glm::mat4 {
    glm::vec3 f,u,r; camBasis(c, f,u,r);
    glm::vec3 o = c.pos;
    return glm::lookAt(o, o + f, u);
};

// Wrap your existing 3D scene drawing (no ImGui calls here)
auto RenderScenePass = [&](){
    // Choose projection/view for this pass
    glm::mat4 P = gUseOverridePV
        ? gOverrideP
        : glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
    glm::mat4 V = gUseOverridePV ? gOverrideV : Cam.view();

    // ---- WATER or DEBUG CUBE (unchanged) ----
    if (!S.debugCube) {
        glm::mat4 MVP=P*V; glUseProgram(progSurf);
        glUniformMatrix4fv(glGetUniformLocation(progSurf,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP));
        glUniform1f(glGetUniformLocation(progSurf,"uAlp"), S.opacity);
        glUniform1f(glGetUniformLocation(progSurf,"uTime"), (float)glfwGetTime());
        // waves
        int n = std::min(S.nDir, 32);
        glUniform1i(glGetUniformLocation(progSurf, "uN"), n);
        for (int i = 0; i < n; ++i) {
            const WaveParams &w = S.waves[i];
            float ang = glm::radians(w.angle_deg);
            glm::vec2 dir = glm::normalize(glm::vec2(std::cos(ang), std::sin(ang)));
            float omega = 2.0f * glm::pi<float>() * w.freq_hz;
            float k     = 2.0f * glm::pi<float>() / std::max(w.lambda_m, 0.001f);
            std::string base = "uDirAmp[" + std::to_string(i) + "]";
            glUniform4f(glGetUniformLocation(progSurf, base.c_str()), dir.x, dir.y, w.height_m, omega);
            base = "uWaveLen[" + std::to_string(i) + "]";
            glUniform2f(glGetUniformLocation(progSurf, base.c_str()), k, 0.0f);
        }
        glBindVertexArray(vaoSurf);  glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
        // side walls
        glBindVertexArray(vaoSides);
        glDrawElements(GL_TRIANGLES, sideIndexCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    } else {
        // --- DEBUG CUBE in main/sub viewport (with transform & selection tint) ---
        glm::mat4 T = glm::translate(glm::mat4(1.f), gCube.pos);
        glm::mat4 R = glm::mat4_cast(gCube.rot);
        glm::mat4 Sct= glm::scale(glm::mat4(1.f), glm::vec3(gCube.scale));
        glm::mat4 M = T * R * Sct;
        glm::mat4 MVP = P * V * M;

        glUseProgram(progCube);
        glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),
                           1, GL_FALSE, glm::value_ptr(MVP));

        // filled cube
        bool mainIsMultiSelected =
            (std::find(gSelCubes.begin(), gSelCubes.end(), 0) != gSelCubes.end());
        ImVec4 base = ((gSelectedCube == 0) || mainIsMultiSelected)
                      ? ImVec4(0.55f,0.68f,0.95f,1.0f)
                      : ImVec4(0.7f,0.7f,0.7f,1.0f);
        glUniform4f(glGetUniformLocation(progCube,"uColor"), base.x, base.y, base.z, base.w);
        glBindVertexArray(vaoCubeFill);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        // outline
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.15f, 0.15f, 0.15f, 1.0f);
        glBindVertexArray(vaoCubeEdge);
        glDrawArrays(GL_LINES, 0, 24);

        // committed copies
        auto drawCube = [&](const CubeState& C, bool selected){
            glm::mat4 T = glm::translate(glm::mat4(1.f), C.pos);
            glm::mat4 R = glm::mat4_cast(C.rot);
            glm::mat4 Sct= glm::scale(glm::mat4(1.f), glm::vec3(C.scale));
            glm::mat4 MVP2 = P * V * (T * R * Sct);

            glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP2));
            ImVec4 col = selected ? ImVec4(0.55f,0.68f,0.95f,1.0f) : ImVec4(0.70f,0.70f,0.70f,1.0f);
            glUniform4f(glGetUniformLocation(progCube,"uColor"), col.x,col.y,col.z,col.w);
            glBindVertexArray(vaoCubeFill);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.15f,0.15f,0.15f, 1.0f);
            glBindVertexArray(vaoCubeEdge);
            glDrawArrays(GL_LINES, 0, 24);
        };
        for (int i = 0; i < (int)gExtraCubes.size(); ++i) {
            int idx = i + 1; // extra cubes are 1-based (0 is the main cube)
            bool isMulti   = (std::find(gSelCubes.begin(), gSelCubes.end(), idx) != gSelCubes.end());
            bool isPrimary = (gSelectedCube == idx);
            drawCube(gExtraCubes[i], isPrimary || isMulti);
        }

        // moving clone
        if (gMove.cloneActive) {
            glm::mat4 T = glm::translate(glm::mat4(1.f), gMove.cloneCube.pos);
            glm::mat4 R = glm::mat4_cast(gMove.cloneCube.rot);
            glm::mat4 Sct= glm::scale(glm::mat4(1.f), glm::vec3(gMove.cloneCube.scale));
            glm::mat4 MVP3 = P * V * (T * R * Sct);

            glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP3));
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.55f,0.68f,0.95f,1.0f);
            glBindVertexArray(vaoCubeFill);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.15f,0.15f,0.15f,1.0f);
            glBindVertexArray(vaoCubeEdge);
            glDrawArrays(GL_LINES, 0, 24);
        }

        // restore polygon mode according to user setting
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
        glBindVertexArray(0);
    }

    // --- NEW: draw imported meshes (solid) ---
    if (!gMeshes.empty()) {
        glUseProgram(progCube);
        for (int i = 0; i < (int)gMeshes.size(); ++i) {
            const auto& Msh = gMeshes[i];
            if (!Msh.visible || Msh.indexCount == 0) continue;

            glm::mat4 T = glm::translate(glm::mat4(1.f), Msh.pos);
            glm::mat4 R = glm::mat4_cast(Msh.rot);
            glm::mat4 Sct= glm::scale(glm::mat4(1.f), glm::vec3(Msh.scale));
            glm::mat4 MVP = P * V * (T * R * Sct);

            glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP));

            // tint selected mesh
            bool isMulti   = (std::find(gSelectedMeshes.begin(), gSelectedMeshes.end(), i) != gSelectedMeshes.end());
            bool isPrimary = (gSelectedMesh == i);
            ImVec4 col = (isPrimary || isMulti) ? ImVec4(0.55f,0.68f,0.95f,1.0f)
                                                : ImVec4(0.70f,0.70f,0.70f,1.0f);
            glUniform4f(glGetUniformLocation(progCube,"uColor"), col.x,col.y,col.z,col.w);

            glBindVertexArray(Msh.vao);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawElements(GL_TRIANGLES, Msh.indexCount, GL_UNSIGNED_INT, nullptr);
        }
        glBindVertexArray(0);

        // restore polygon mode according to user setting
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
    }

    // ---- World axes (lines only; labels are in overlay later) ----
    if (S.showAxes) {
        float w = std::max(0.001f, S.width_m);
        float l = std::max(0.001f, S.length_m);
        float d = std::max(0.001f, S.depth_m);
        float L = std::max(0.5f * std::sqrt(w*w + l*l), 0.5f * d) * 1.2f;

        glm::vec3 a[6] = {
            {-L, 0, 0}, {+L, 0, 0},   // X
            { 0,-L, 0}, { 0,+L, 0},   // Y
            { 0, 0,-L}, { 0, 0,+L}    // Z
        };

        if (!vaoAxes) glGenVertexArrays(1,&vaoAxes);
        if (!vboAxes) glGenBuffers(1,&vboAxes);
        glBindVertexArray(vaoAxes);
        glBindBuffer(GL_ARRAY_BUFFER, vboAxes);
        glBufferData(GL_ARRAY_BUFFER, sizeof(a), a, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

        glUseProgram(progCube);
        glm::mat4 MVPaxes = P * V;
        glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),
                           1, GL_FALSE, glm::value_ptr(MVPaxes));
        glBindVertexArray(vaoAxes);
        glLineWidth(2.0f);
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 1.0f, 0.2f, 0.2f, 1.0f); glDrawArrays(GL_LINES, 0, 2);
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.2f, 1.0f, 0.2f, 1.0f); glDrawArrays(GL_LINES, 2, 2);
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.2f, 0.6f, 1.0f, 1.0f); glDrawArrays(GL_LINES, 4, 2);
        glLineWidth(1.0f);
        glBindVertexArray(0);
    }

    // ---- Viewport anchors (green balls) ----
    {
        float R = 0.02f * std::max({ S.width_m, S.length_m, S.depth_m });
        R = std::clamp(R, 0.08f, 1.5f);

        glUseProgram(progCube);
        glBindVertexArray(vaoSphere);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        for (const auto& a : gAnchors) {
            if (!a.visible) continue;
            glm::mat4 M = glm::translate(glm::mat4(1.f), a.pos) * glm::scale(glm::mat4(1.f), glm::vec3(R));
            glm::mat4 MVP = P * V * M;
            glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"), 1, GL_FALSE, glm::value_ptr(MVP));
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.15f, 0.95f, 0.25f, 1.0f);
            glDrawElements(GL_TRIANGLES, sphereIndexCount, GL_UNSIGNED_INT, nullptr);
        }

        glBindVertexArray(0);
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
    }
// ---- Draw scene cameras (lines + orientation triangle) ----
// Hidden inside camera sub-views to avoid yellow rect/triangle in the camera's image.
if (!gInCamSubview) {
    if (!gSceneCams.empty()) {
        glUseProgram(progCube);

        static GLuint vaoCamLines=0, vboCamLines=0;
        static GLuint vaoCamTri=0,   vboCamTri=0;
        if (!vaoCamLines) glGenVertexArrays(1,&vaoCamLines);
        if (!vboCamLines) glGenBuffers(1,&vboCamLines);
        if (!vaoCamTri)   glGenVertexArrays(1,&vaoCamTri);
        if (!vboCamTri)   glGenBuffers(1,&vboCamTri);

        float span = std::max({ S.width_m, S.length_m, S.depth_m, 1.0f });
        float L = 0.15f * span;                // frustum depth
        float halfW = 0.35f * L, halfH = 0.26f * L;
        float triUp = 0.55f * halfH;
        float triHalf = 0.45f * halfW;

        for (const auto& c : gSceneCams) {
            if (!c.visible) continue;

            glm::vec3 f,u,r; camBasis(c, f,u,r);
            glm::vec3 o = c.pos;
            glm::vec3 center = o + f * L;

            glm::vec3 c1 = center + r*halfW + u*halfH;
            glm::vec3 c2 = center - r*halfW + u*halfH;
            glm::vec3 c3 = center - r*halfW - u*halfH;
            glm::vec3 c4 = center + r*halfW - u*halfH;

            // Lines
            glm::vec3 lines[] = { c1,c2, c2,c3, c3,c4, c4,c1,  o,c1, o,c2, o,c3, o,c4 };
            glBindVertexArray(vaoCamLines);
            glBindBuffer(GL_ARRAY_BUFFER, vboCamLines);
            glBufferData(GL_ARRAY_BUFFER, sizeof(lines), lines, GL_DYNAMIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

            glm::mat4 MVP = (gUseOverridePV ? gOverrideP * gOverrideV : glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f) * Cam.view());
            glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP));
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.95f, 0.75f, 0.25f, 1.0f);
            glDrawArrays(GL_LINES, 0, 16);

            // Triangle
            glm::vec3 top = center + u*(halfH + triUp);
            glm::vec3 tl  = center - r*triHalf + u*(halfH + triUp*0.2f);
            glm::vec3 tr  = center + r*triHalf + u*(halfH + triUp*0.2f);
            glm::vec3 tri[3] = { top, tl, tr };

            glBindVertexArray(vaoCamTri);
            glBindBuffer(GL_ARRAY_BUFFER, vboCamTri);
            glBufferData(GL_ARRAY_BUFFER, sizeof(tri), tri, GL_DYNAMIC_DRAW);
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.95f, 0.60f, 0.15f, 1.0f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawArrays(GL_TRIANGLES, 0, 3);
            glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
        }
        glBindVertexArray(0);
    }
}

};

// ---------------- Main + Subviews ----------------
glViewport(0,0,FBW,FBH);
glClearColor(S.dark?0.05f:1.0f, S.dark?0.07f:1.0f, S.dark?0.1f:1.0f, 1.0f);
glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

// Main (sim) view
gUseOverridePV = false;
gInCamSubview  = false;
RenderScenePass();

    // Camera sub-views: render each camera into its own texture (works in popped-out windows)
    for (CamViewWin& cv : gCamViews) {
        if (!cv.open || cv.camIndex < 0 || cv.camIndex >= (int)gSceneCams.size()) continue;

        // Target size from UI (fallback for first frame)
        int w = (cv.wantW > 0) ? cv.wantW : 320;
        int h = (cv.wantH > 0) ? cv.wantH : 180;

        ensureCamRT(cv, w, h);

        glBindFramebuffer(GL_FRAMEBUFFER, cv.rtFBO);
        glViewport(0, 0, cv.rtW, cv.rtH);
        glEnable(GL_DEPTH_TEST);
        glClearColor(S.dark ? 0.05f : 1.0f, S.dark ? 0.07f : 1.0f, S.dark ? 0.10f : 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const SceneCam& C = gSceneCams[cv.camIndex];
        float aspect = (cv.rtH > 0) ? float(cv.rtW) / float(cv.rtH) : 1.0f;

        // Use camera's FOV (already kept in sync with focal & sensor height)
        gOverrideP = glm::perspective(glm::radians(C.fov_deg), aspect, 0.1f, 2000.f);
        gOverrideV = viewFromSceneCam(C);

        gUseOverridePV = true;
        gInCamSubview  = true;
        RenderScenePass();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
    // restore flags (main view uses its own PV)
    gUseOverridePV = false;
    gInCamSubview  = false;

    ImGui_ImplOpenGL3_NewFrame();   // <-- ADD THIS
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    settingsUI();  // updates gSettingsBottomY / gSettingsCollapsed

    // --- Move gizmo overlay (always-on-top) ---
    if (gMove.active && haveSelection()) {
        // Use the main viewport as the coordinate origin for ImGui overlay
#ifdef IMGUI_HAS_VIEWPORT
        ImGuiViewport* mainvp = ImGui::GetMainViewport();      // <-- non-const
        ImDrawList* dl = ImGui::GetForegroundDrawList(mainvp);
        ImVec2 vpOrigin = mainvp->Pos;
#else
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        ImVec2 vpOrigin = ImVec2(0,0);
#endif


        // Project pivot to screen (framebuffer pixels), then offset by viewport origin
        glm::mat4 P = glm::perspective(glm::radians(45.f), float(FBW)/FBH, 0.1f, 2000.f);
        glm::mat4 V = Cam.view();

        auto worldToScreen = [&](const glm::vec3& w)->ImVec2{
            glm::vec4 c = P * V * glm::vec4(w,1);
            glm::vec3 ndc = glm::vec3(c) / c.w;
            // framebuffer pixel coords
            float sx = (ndc.x * 0.5f + 0.5f) * float(FBW);
            float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * float(FBH);
            // convert to viewport-relative ImGui coords
            return ImVec2(vpOrigin.x + sx, vpOrigin.y + sy);
        };

        glm::vec3 pw = currentPivotWorldPos();
        glm::mat3 B  = currentPivotWorldBasis();

        ImVec2 p = worldToScreen(pw);
    // Larger, clearer handles
    const float pxLen   = 140.f;
    const float pxShort = 100.f;
    const float hitPix  = 10.f;

    // Axis colors (high contrast) + outline
    auto C = [](int r,int g,int b,int a=255){ return IM_COL32(r,g,b,a); };
    const ImU32 colX = C(235, 80, 80);
    const ImU32 colY = C( 90,210, 90);
    const ImU32 colZ = C( 90,140,255);
    const ImU32 colHi = C(255,255,255);
    const ImU32 colPlaneFill = C(180,180,180,60);
    const ImU32 colPlaneHi   = C(120,180,255,90);
    const ImU32 colOutline   = C(  0,  0,  0,180);

    auto axisEnd = [&](const glm::vec3& dir)->ImVec2{
        glm::vec4 c = P * V * glm::vec4(pw,1);
        float s = (pxLen / float(FBH)) * c.w * 2.0f * std::tan(glm::radians(45.f)*0.5f);
        return worldToScreen(pw + glm::normalize(dir) * s);
    };

    ImVec2 ex = axisEnd(B[0]);
    ImVec2 ey = axisEnd(B[1]);
    ImVec2 ez = axisEnd(B[2]);

    auto distToSeg = [](ImVec2 a, ImVec2 b, ImVec2 p)->float{
        ImVec2 ab(b.x-a.x, b.y-a.y), ap(p.x-a.x, p.y-a.y);
        float t = (ab.x*ap.x + ab.y*ap.y) / (ab.x*ab.x + ab.y*ab.y + 1e-6f);
        t = std::clamp(t, 0.f, 1.f);
        ImVec2 q(a.x + t*ab.x, a.y + t*ab.y);
        float dx = q.x - p.x, dy = q.y - p.y;
        return std::sqrt(dx*dx+dy*dy);
    };

    // Only compute hover when not dragging
    if (!gMove.dragging) gMove.hot = MoveState::None;
    ImVec2 mp(ImGui::GetIO().MousePos.x, ImGui::GetIO().MousePos.y);

    auto drawArrow = [&](ImVec2 e, ImU32 col, MoveState::Handle h){
        // outline then colored line (for contrast)
        dl->AddLine(p, e, colOutline, 6.0f);
        dl->AddLine(p, e, col,        3.0f);

        // arrow head (outlined)
        ImVec2 v(e.x-p.x, e.y-p.y); float L = std::sqrt(v.x*v.x+v.y*v.y)+1e-5f; v.x/=L; v.y/=L;
        ImVec2 left (e.x - v.x*14.f - v.y*9.f,  e.y - v.y*14.f + v.x*9.f);
        ImVec2 right(e.x - v.x*14.f + v.y*9.f,  e.y - v.y*14.f - v.x*9.f);
        dl->AddTriangleFilled(e, left, right, colOutline);
        ImVec2 leftIn (e.x - v.x*12.f - v.y*7.f,  e.y - v.y*12.f + v.x*7.f);
        ImVec2 rightIn(e.x - v.x*12.f + v.y*7.f,  e.y - v.y*12.f - v.x*7.f);
        dl->AddTriangleFilled(e, leftIn, rightIn, col);

        if (!gMove.dragging && distToSeg(p,e,mp) < hitPix) gMove.hot = h;
    };

    auto mixed = [&](ImU32 a, ImU32 b)->ImU32{
        ImVec4 A = ImGui::ColorConvertU32ToFloat4(a);
        ImVec4 B = ImGui::ColorConvertU32ToFloat4(b);
        ImVec4 M((A.x+B.x)*0.5f,(A.y+B.y)*0.5f,(A.z+B.z)*0.5f,(A.w+B.w)*0.5f);
        return ImGui::ColorConvertFloat4ToU32(M);
    };

    // Draw axes (Translate or Free)
    bool showAxis = (gMove.type==MoveType::Free || gMove.type==MoveType::Translate);
    bool showArc  = (gMove.type==MoveType::Free || gMove.type==MoveType::Rotate) && !anchorMode();
    if (showAxis) {
        drawArrow(ex, (gMove.hot==MoveState::AxisX)?colHi:colX, MoveState::AxisX);
        drawArrow(ey, (gMove.hot==MoveState::AxisY)?colHi:colY, MoveState::AxisY);
        drawArrow(ez, (gMove.hot==MoveState::AxisZ)?colHi:colZ, MoveState::AxisZ);
    }

    // Plane squares
    auto drawPlane = [&](ImVec2 e1, ImVec2 e2, MoveState::Handle h, ImU32 colA, ImU32 colB){
        ImU32 fill = (!gMove.dragging && gMove.hot==h)?colPlaneHi:colPlaneFill;
        ImU32 edge = mixed(colA, colB);

        ImVec2 a = ImVec2(p.x + (e1.x-p.x)*0.35f, p.y + (e1.y-p.y)*0.35f);
        ImVec2 b = ImVec2(a.x + (e2.x-p.x)*0.35f, a.y + (e2.y-p.y)*0.35f);
        ImVec2 c = ImVec2(p.x + (e2.x-p.x)*0.35f, p.y + (e2.y-p.y)*0.35f);

        dl->AddTriangleFilled(p,a,c,fill);
        dl->AddTriangleFilled(a,b,c,fill);
        dl->AddTriangle(p,a,c,edge,1.6f);
        dl->AddTriangle(a,b,c,edge,1.6f);

        auto cross2 = [](ImVec2 u, ImVec2 v){ return u.x*v.y - u.y*v.x; };
        auto insideTri = [&](ImVec2 A, ImVec2 B, ImVec2 C, ImVec2 P)->bool{
            ImVec2 v0(C.x-A.x,C.y-A.y), v1(B.x-A.x,B.y-A.y), v2(P.x-A.x,P.y-A.y);
            float d00 = v0.x*v0.x+v0.y*v0.y;
            float d01 = v0.x*v1.x+v0.y*v1.y;
            float d11 = v1.x*v1.x+v1.y*v1.y;
            float d20 = v2.x*v0.x+v2.y*v0.y;
            float d21 = v2.x*v1.x+v2.y*v1.y;
            float denom = d00*d11 - d01*d01 + 1e-6f;
            float v = (d11*d20 - d01*d21)/denom;
            float w = (d00*d21 - d01*d20)/denom;
            float u = 1.0f - v - w;
            return u>=0 && v>=0 && w>=0;
        };
        if (!gMove.dragging && (insideTri(p,a,c,mp) || insideTri(a,b,c,mp))) gMove.hot = h;
    };

    if (showAxis) {
        drawPlane(ex, ey, MoveState::PlaneXY, colX, colY);
        drawPlane(ex, ez, MoveState::PlaneXZ, colX, colZ);
        drawPlane(ey, ez, MoveState::PlaneYZ, colY, colZ);
    }

    // --- 60° Arc segments (3D in local planes, then projected) ---
    auto rotAround = [](const glm::vec3& dir, const glm::vec3& n, float ang)->glm::vec3{
        return glm::normalize(glm::angleAxis(ang, glm::normalize(n)) * dir);
    };

    // Convert desired pixel radius to a local world-space radius near pivot
    auto worldRadiusForPixels = [&](float pixels)->float{
        glm::vec4 c = P * V * glm::vec4(pw, 1.0f);                 // clip-space center
        float fovy = glm::radians(45.f);
        // Scale like our axis-end computation so it tracks perspective depth
        return (pixels / float(FBH)) * c.w * 2.0f * std::tan(fovy * 0.5f);
    };

    auto drawArc3D = [&](const glm::vec3& uW, const glm::vec3& vW,
                         MoveState::Handle handle, ImU32 col){
        // Plane basis & normal
        glm::vec3 u = glm::normalize(uW);
        glm::vec3 v = glm::normalize(vW);
        glm::vec3 n = glm::normalize(glm::cross(u, v));

        // 60° span around the bisector of the two axes
        float span = glm::radians(60.f);
        glm::vec3 mdir = glm::normalize(u + v);        // bisector in the plane
        glm::vec3 d0   = rotAround(mdir, n, -span*0.5f);

        // World radius sized to ~pxShort on screen
        float rW = worldRadiusForPixels(pxShort);

        // Polyline sample in world, project to screen, draw with outline+color
        int seg = 28;
        ImVec2 prev;
        bool hasPrev = false;
        float bestHit = 1e9f;

        for (int i = 0; i <= seg; ++i) {
            float t = (i / float(seg)) * span;
            glm::vec3 d = rotAround(d0, n, t);
            ImVec2 sp = worldToScreen(pw + d * rW);

            if (hasPrev) {
                dl->AddLine(prev, sp, colOutline, 5.0f);
                dl->AddLine(prev, sp, col,        2.2f);

                if (!gMove.dragging) {
                    float dd = distToSeg(prev, sp, mp);
                    if (dd < bestHit) bestHit = dd;
                }
            }
            prev = sp;
            hasPrev = true;
        }

        if (!gMove.dragging && bestHit < hitPix + 2.f)
            gMove.hot = handle;
    };

    if (showArc) {
        // ArcZ: rotate about B[2], drawn in XY plane (B[0], B[1])
        drawArc3D(B[0], B[1],
                  MoveState::ArcZ,
                  (gMove.hot==MoveState::ArcZ)?colHi:colZ);
        // ArcX: rotate about B[0], drawn in YZ plane (B[1], B[2])
        drawArc3D(B[1], B[2],
                  MoveState::ArcX,
                  (gMove.hot==MoveState::ArcX)?colHi:colX);
        // ArcY: rotate about B[1], drawn in ZX plane (B[2], B[0])
        drawArc3D(B[2], B[0],
                  MoveState::ArcY,
                  (gMove.hot==MoveState::ArcY)?colHi:colY);
    }

    // center dot
    dl->AddCircleFilled(p, 5.f, colOutline);
    dl->AddCircleFilled(p, 3.f, C(220,220,220,230));
}

    // ---- Axes labels (overlay in screen space) ----
    if (S.showAxes) {
        // Recompute matrices locally for the overlay
        glm::mat4 P_lbl  = glm::perspective(glm::radians(45.f), float(FBW)/float(FBH), 0.1f, 2000.f);
        glm::mat4 V_lbl  = Cam.view();
        glm::mat4 PV_lbl = P_lbl * V_lbl;

        auto drawLabel = [&](const glm::vec3& p, const char* s){
            glm::vec4 c = PV_lbl * glm::vec4(p,1.0f);
            if (c.w <= 0.0f) return; // behind camera
            glm::vec3 ndc = glm::vec3(c) / c.w;
            if (ndc.z < -1.0f || ndc.z > 1.0f) return;
            float sx = (ndc.x * 0.5f + 0.5f) * float(FBW);
            float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * float(FBH);
            ImDrawList* dl = ImGui::GetForegroundDrawList();
            ImU32 col = IM_COL32(255,255,255,230);
            ImU32 shd = IM_COL32(0,0,0,160);
            dl->AddText(ImVec2(sx+1, sy+1), shd, s);
            dl->AddText(ImVec2(sx,   sy  ), col, s);
        };

        float w = std::max(0.001f, S.width_m);
        float l = std::max(0.001f, S.length_m);
        float d = std::max(0.001f, S.depth_m);
        float L = std::max(0.5f * std::sqrt(w*w + l*l), 0.5f * d) * 1.2f;

        if (!gInCamSubview) {
            drawLabel(glm::vec3( L, 0, 0), "X");
            drawLabel(glm::vec3( 0, L, 0), "Y");
            drawLabel(glm::vec3( 0, 0, L), "Z");
        }
    }

        // ---------- cube overlay ----------
        glDisable(GL_DEPTH_TEST);

        // Compute placement now that Settings UI size/pos are up to date
        computeCubePlacement();

        // Set cube viewport (GL coords are bottom-left)
        glViewport(gCubeX, FBH - CubePx - gCubeY, CubePx, CubePx);

        // Build MVP for the cube (must match picker)
        glm::mat4 cubeMVP = makeCubeMVP();

        /* --- determine hovered cube face ----------------------------------- */
        double mxCur, myCur; glfwGetCursorPos(Win, &mxCur, &myCur);
        cubeHoverFace = inCubeRect(mxCur, myCur) ? pickCubeFaceAt(mxCur, myCur) : -1;

        /* -------- draw cube wireframe -------- */
        glUseProgram(progCube);
        glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),
                           1, GL_FALSE, glm::value_ptr(cubeMVP));
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.6f, 0.6f, 0.6f, 1.0f);
        glBindVertexArray(vaoCubeEdge);
        glDrawArrays(GL_LINES, 0, 24);          // 12 edges

        /* -------- highlight hovered face (solid, 50% alpha) -------- */
        if (cubeHoverFace >= 0) {
            static const int first[6] = { 0, 6, 12, 18, 24, 30 };  // 6 verts each
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.6f,0.6f,0.6f,0.5f);
            glBindVertexArray(vaoCubeFill);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawArrays(GL_TRIANGLES, first[cubeHoverFace], 6);
        }

    // -------- draw "Home" (fit view) button next to the cube ------------
    {
    float x0,y0,x1,y1; getHomeBtnRect(x0,y0,x1,y1);  // absolute ImGui coords
    const bool hovered = inHomeBtnRect();

    // Visual-only tweak: nudge the drawn icon down a bit (hitbox stays exact)
    const float drawYOffset = 2.0f;

    ImDrawList* dl = ImGui::GetForegroundDrawList(ImGui::GetMainViewport());
    ImU32 colFill = hovered ? IM_COL32(230,230,230,192) : IM_COL32(200,200,200,128);
    ImU32 colLine = hovered ? IM_COL32( 60, 60, 60,224) : IM_COL32( 40, 40, 40,160);

    float w = x1 - x0, h = y1 - y0, xc = (x0 + x1) * 0.5f;
    float pad   = 3.0f;
    float roofH = h * 0.45f;
    float bodyL = x0 + pad + 3.0f;
    float bodyR = x1 - pad - 3.0f;
    float bodyT = (y0 + roofH) + drawYOffset;
    float bodyB = (y1 - pad)   + drawYOffset;

    ImVec2 pTop (xc, y0 + pad + drawYOffset);
    ImVec2 pL   (x0 + pad, bodyT);
    ImVec2 pR   (x1 - pad, bodyT);
    dl->AddTriangleFilled(pTop, pL, pR, colFill);
    dl->AddTriangle(pTop, pL, pR, colLine, 1.5f);

    dl->AddRectFilled(ImVec2(bodyL, bodyT), ImVec2(bodyR, bodyB), colFill, 2.5f);
    dl->AddRect(ImVec2(bodyL, bodyT), ImVec2(bodyR, bodyB), colLine, 2.5f, 0, 1.5f);
    }



        /* restore state */
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
        glBindVertexArray(0);

        glViewport(0,0,FBW,FBH);
        glEnable(GL_DEPTH_TEST);


        // Render GUI on top
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Render additional OS windows (multi-viewport)
#ifdef IMGUI_HAS_VIEWPORT
    if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
        GLFWwindow* backup_ctx = glfwGetCurrentContext();
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
        glfwMakeContextCurrent(backup_ctx);
    }
#endif


        glfwSwapBuffers(Win); }

    }catch(std::exception& e){ std::cerr<<"Fatal: "<<e.what()<<std::endl; }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext(); glfwDestroyWindow(Win); glfwTerminate(); return 0; }