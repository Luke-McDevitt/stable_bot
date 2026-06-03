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
    // domain
    float width_m=50.f, length_m=50.f, depth_m=20.f, visc=0.001f;
    // waves
    int nDir=1; bool linear=true; std::vector<WaveParams> waves;
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


//---------------------------------- GL objects ---------------------------------
static GLuint progSurf=0, progCube=0;
static GLuint vaoSurf=0,vboSurf=0,eboSurf=0;   // ← keep
static size_t idxCount=0;
/* cube: one VAO/VBO for wire‑edges, one for solid faces  */
static GLuint vaoCubeEdge = 0, vboCubeEdge = 0;
static GLuint vaoCubeFill = 0, vboCubeFill = 0;

static GLuint vaoSides=0, vboSides=0, eboSides=0;
static GLsizei sideIndexCount = 0;
static int   cubeHoverFace = -1;     // 0‑5 if mouse over, -1 otherwise

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


//---------------------------------- ImGui panel --------------------------------
static void settingsUI(){ ImGui::SetNextWindowPos(ImVec2(FBW-10,10),ImGuiCond_Always,ImVec2(1,0)); ImGui::Begin("Settings",nullptr,ImGuiWindowFlags_NoMove|ImGuiWindowFlags_AlwaysAutoResize);
    if(ImGui::Checkbox("Dark mode", &S.dark)){}
    if(ImGui::Checkbox("Wireframe", &S.wire)) glPolygonMode(GL_FRONT_AND_BACK,S.wire?GL_LINE:GL_FILL);
    if(S.wire && ImGui::SliderInt("Res 2^n", &S.res_log2, 4, 11)) buildSurface();
    float op=S.opacity*100.f;
    if(ImGui::SliderFloat("Opacity %", &op, 0.0f, 100.0f, "%.1f"))
        S.opacity = op/100.f;
    if(ImGui::InputFloat("Width (m)", &S.width_m) | ImGui::InputFloat("Length (m)", &S.length_m)) buildSurface();
    ImGui::InputFloat("Depth (m)", &S.depth_m); ImGui::InputFloat("Viscosity", &S.visc);
    ImGui::Checkbox("Linear superposition", &S.linear);
    if(ImGui::InputInt("Wave directions", &S.nDir)){ S.nDir=std::clamp(S.nDir,1,32); S.waves.resize(S.nDir);}
    ImGui::BeginChild("wavechild", ImVec2(0,160), true, ImGuiWindowFlags_HorizontalScrollbar);
    for(int i=0;i<S.nDir;++i){ ImGui::PushID(i); ImGui::Text("Dir %d",i); ImGui::SameLine();
        ImGui::InputFloat("Angle", &S.waves[i].angle_deg);
        ImGui::InputFloat("Height", &S.waves[i].height_m);
        ImGui::InputFloat("Freq", &S.waves[i].freq_hz);
        ImGui::InputFloat("λ", &S.waves[i].lambda_m);
        ImGui::InputFloat("Decay", &S.waves[i].decay_s);
        ImGui::Separator(); ImGui::PopID(); }
    ImGui::EndChild(); ImGui::End(); }

//---------------------------------- Input callbacks ----------------------------
static bool ioCaptures(){ ImGuiIO& io=ImGui::GetIO(); return io.WantCaptureMouse || io.WantCaptureKeyboard; }
static void cursorCB(GLFWwindow* w, double x, double y)
{
    double dx = x - lastX, dy = y - lastY;
    lastX = x; lastY = y;
    if (ioCaptures()) return;

    const float rot = 0.005f;
    const float pan = 0.01f;

    // Live modifier query (don’t rely on stale SHIFT from mouseBtnCB)
    const bool shiftHeld =
        glfwGetKey(w, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
        glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    if (MMB && shiftHeld) {
        // ORBIT (rotate) – same feel as your old LMB orbit:
        // drag left => Cam.yaw increases (model looks to spin clockwise)
        Cam.yaw   -= rot * dx;
        Cam.pitch += rot * dy;
        Cam.pitch  = std::clamp(Cam.pitch, -1.55f, 1.55f);
    }
    else if (MMB) {
        // PAN – unchanged
        glm::vec3 right = glm::normalize(glm::cross(
            glm::vec3(0,1,0),
            glm::vec3(std::sin(Cam.yaw), 0, std::cos(Cam.yaw))
        ));
        glm::vec3 up(0,1,0);
        Cam.pan -= right * float(dx) * pan * Cam.dist * 0.05f;
        Cam.pan += up    * float(dy) * pan * Cam.dist * 0.05f;
    }
    else {
        // No M1 navigation: ignore LMB drags entirely (but clicks still work for the cube)
    }
}


// cube face clicking functionality
static const int cubePx = 120;          // cube viewport size (px)


/* mouse button callback -----------------------------------------------------*/
static void mouseBtnCB(GLFWwindow*, int button, int action, int mods)
{
    SHIFT = mods & GLFW_MOD_SHIFT;
    if (button == GLFW_MOUSE_BUTTON_LEFT)   LMB = (action == GLFW_PRESS);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) MMB = (action == GLFW_PRESS);

    /* ── cube‑face pick ──────────────────────────────────────────────────── */
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        double mx, my; glfwGetCursorPos(Win, &mx, &my);

        int cubeX = 20;
        int cubeY = 20;
        if (mx >= cubeX && mx <= cubeX + cubePx &&
            my >= cubeY && my <= cubeY + cubePx)
        {
            /* 1 · mouse → NDC in cube viewport */
            float nx = float(mx - cubeX) / cubePx * 2.0f - 1.0f;
            float ny = float(cubePx - (my - cubeY)) / cubePx * 2.0f - 1.0f;

            /* 2 · rebuild the same cube MVP used for drawing */
            glm::mat4 cubeP = glm::ortho(-1.f, 1.f, -1.f, 1.f, 0.1f, 5.f);
            glm::mat4 cubeR = glm::mat4(glm::mat3(Cam.view()));          // only rotation
            glm::mat4 cubeS = glm::scale(glm::mat4(1.f), glm::vec3(0.8f));
            glm::mat4 cubeT = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -2.f));
            glm::mat4 cubeMVP = cubeP * cubeR * cubeT * cubeS;

            /* --- 3 · cast a ray and find the first cube face it hits ------------- */
            glm::mat4 inv = glm::inverse(cubeMVP);

            /* ray through the pixel: NDC near (z=-1) → far (z=+1) */
            glm::vec3 pNear = glm::vec3(inv * glm::vec4(nx, ny, -1.0f, 1.0f));
            glm::vec3 pFar  = glm::vec3(inv * glm::vec4(nx, ny,  1.0f, 1.0f));
            glm::vec3 rayO  = pNear;
            glm::vec3 rayD  = glm::normalize(pFar - pNear);

            /* axis‑aligned cube in cube space has half‑size 0.4 (scale 0.8) */
            const float h = 0.375f;           // 0.75 (scale) × 0.5

            /* compute t at which ray hits each slab */
            auto hitT = [&](float o, float d, float sgn){
                return (sgn * h - o) / d;      // t to +h (sgn=+1) or –h (sgn=‑1)
            };

            float bestT = 1e9f; int bestAxis = -1; float bestSgn = 0;

            for (int axis = 0; axis < 3; ++axis) {
                float o = rayO[axis], d = rayD[axis];
                if (fabs(d) < 1e-4f) continue;                // parallel to slab
                for (float sgn : { -1.f, 1.f }) {
                    float t = hitT(o, d, sgn);
                    if (t > 0.0f && t < bestT) {              // nearest positive hit
                        bestT = t; bestAxis = axis; bestSgn = sgn;
                    }
                }
            }

            /* --- 4 · snap camera based on hit face -------------------------------- */
            if (bestAxis == 0) {              // ±X
                Cam.yaw   = (bestSgn > 0) ? 0.0f : glm::pi<float>();
                Cam.pitch = 0.0f;
            }
            else if (bestAxis == 1) {         // ±Y  (top/bottom)
                Cam.yaw   = 0.0f;             // keep current yaw if preferred
                Cam.pitch = (bestSgn > 0) ? -glm::half_pi<float>()
                                          :  glm::half_pi<float>();
            }
            else if (bestAxis == 2) {         // ±Z
                Cam.yaw   = (bestSgn > 0) ?  glm::half_pi<float>()
                                          : -glm::half_pi<float>();
                Cam.pitch = 0.0f;
            }


            // clicking top / bottom keeps current yaw; could add logic if desired
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
    const int cubeX = 20, cubeY = 20, cubePxLocal = 120;
    const bool overCube = (mx >= cubeX && mx <= cubeX + cubePxLocal &&
                           my >= cubeY && my <= cubeY + cubePxLocal);

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
    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGui_ImplGlfw_InitForOpenGL(Win,true); ImGui_ImplOpenGL3_Init("#version 450"); ImGui::StyleColorsDark();
    progSurf=link({comp(GL_VERTEX_SHADER,SurfVS), comp(GL_FRAGMENT_SHADER,SurfFS)});
    progCube=link({comp(GL_VERTEX_SHADER,CubeVS), comp(GL_FRAGMENT_SHADER,CubeFS)});
    buildSurface(); buildCube(); S.waves.resize(1); }

int main(){ try{ init(); while(!glfwWindowShouldClose(Win)){ glfwPollEvents(); glfwGetFramebufferSize(Win,&FBW,&FBH);
        glViewport(0,0,FBW,FBH); glClearColor(S.dark?0.05f:1.0f, S.dark?0.07f:1.0f, S.dark?0.1f:1.0f, 1.0f); glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        // water surface
        glm::mat4 P=glm::perspective(glm::radians(45.f), float(FBW)/FBH, 0.1f, 2000.f);
        glm::mat4 V=Cam.view(); glm::mat4 MVP=P*V; glUseProgram(progSurf);
        glUniformMatrix4fv(glGetUniformLocation(progSurf,"uMVP"),1,GL_FALSE,glm::value_ptr(MVP));
        glUniform1f(glGetUniformLocation(progSurf,"uAlp"), S.opacity);
        glUniform1f(glGetUniformLocation(progSurf,"uTime"), (float)glfwGetTime());
        //---------------- upload wave uniforms ----------------
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
        glBindVertexArray(vaoSurf);
        glDrawElements(GL_TRIANGLES, idxCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
        // side walls
        glBindVertexArray(vaoSides);
        glDrawElements(GL_TRIANGLES, sideIndexCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);

        // ---------- cube overlay ----------
        glDisable(GL_DEPTH_TEST);
        const int cubePx = 120;                // ← pick a size you like
        glViewport(20, FBH - cubePx - 20, cubePx, cubePx);
        /* --- build centred cube MVP ----------------------------------------- */
        glm::mat4 cubeP = glm::ortho(-1.f, 1.f, -1.f, 1.f, 0.1f, 6.f);   // farther far‑plane
        glm::mat4 cubeR = glm::mat4(glm::mat3(V));                       // camera yaw/pitch
        glm::mat4 cubeS = glm::scale(glm::mat4(1.f),  glm::vec3(0.75f)); // bigger but fits
        /* translate straight back along camera -Z AFTER rotation is applied */
        glm::mat4 cubeT = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, -1.8f));
        /* final MVP:  P * T * R * S  (S first, then rotate, then push back) */
        glm::mat4 cubeMVP = cubeP * cubeT * cubeR * cubeS;


        /* --- determine hovered cube face ----------------------------------- */
        double mxCur, myCur; glfwGetCursorPos(Win, &mxCur, &myCur);
        if (mxCur >= 20 && mxCur <= 20 + cubePx &&
            myCur >= 20 && myCur <= 20 + cubePx)
        {
            float nx = float(mxCur - 20) / cubePx * 2.0f - 1.0f;
            float ny = float(cubePx - (myCur - 20)) / cubePx * 2.0f - 1.0f;

            glm::mat4 inv = glm::inverse(cubeMVP);
            glm::vec3 pNear = glm::vec3(inv * glm::vec4(nx, ny, -1.0f, 1.0f));
            glm::vec3 pFar  = glm::vec3(inv * glm::vec4(nx, ny,  1.0f, 1.0f));
            glm::vec3 rayO  = pNear;
            glm::vec3 rayD  = glm::normalize(pFar - pNear);

            const float h = 0.375f;                     // half‑size
            float bestT = 1e9f; int bestAxis = -1; float bestSgn = 0.f;

            for (int axis = 0; axis < 3; ++axis) {
                float o = rayO[axis], d = rayD[axis];
                if (fabs(d) < 1e-4f) continue;                 // ray ‖ slab
                for (float s : { -1.f, 1.f }) {
                    float t = (s * h - o) / d;                 // hit that plane
                    if (t <= 0.0f || t >= bestT) continue;     // behind ray or farther away
                    glm::vec3 p = rayO + t * rayD;             // intersection point
                    /* check the other two coordinates are inside the face */
                    int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                    if (fabs(p[a1]) <= h + 1e-4f && fabs(p[a2]) <= h + 1e-4f) {
                        bestT = t; bestAxis = axis; bestSgn = s;
                    }
                }
            }

            cubeHoverFace = (bestAxis < 0) ? -1 :
                (bestAxis == 0 ? (bestSgn > 0 ? 0 : 1) :
                 bestAxis == 1 ? (bestSgn > 0 ? 2 : 3) :
                                 (bestSgn > 0 ? 4 : 5));
        }
        else cubeHoverFace = -1;

        /* -------- draw cube wireframe -------- */
        glUseProgram(progCube);
        glUniformMatrix4fv(glGetUniformLocation(progCube,"uMVP"),
                           1, GL_FALSE, glm::value_ptr(cubeMVP));
        glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.6f, 0.6f, 0.6f, 1.0f);
        glBindVertexArray(vaoCubeEdge);
        glDrawArrays(GL_LINES, 0, 24);          // 12 edges

        /* -------- highlight hovered face (solid, 50 % alpha) -------- */
        if (cubeHoverFace >= 0) {
            static const int first[6] = { 0, 6, 12, 18, 24, 30 };  // 6 verts each
            glUniform4f(glGetUniformLocation(progCube,"uColor"), 0.6f,0.6f,0.6f,0.5f);
            glBindVertexArray(vaoCubeFill);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDrawArrays(GL_TRIANGLES, first[cubeHoverFace], 6);
        }

        /* restore state */
        glPolygonMode(GL_FRONT_AND_BACK, S.wire ? GL_LINE : GL_FILL);
        glBindVertexArray(0);

        glViewport(0,0,FBW,FBH);
        glEnable(GL_DEPTH_TEST);

        // GUI
        ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame(); settingsUI(); ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); glfwSwapBuffers(Win); }
    }catch(std::exception& e){ std::cerr<<"Fatal: "<<e.what()<<std::endl; }
    ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown(); ImGui::DestroyContext(); glfwDestroyWindow(Win); glfwTerminate(); return 0; }
