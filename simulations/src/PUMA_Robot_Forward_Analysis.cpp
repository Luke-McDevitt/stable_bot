
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <iostream>

#include <Eigen/Dense>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <fstream>
#include <sstream>
#include <filesystem>
#include <cstdint>
#include <unordered_map>
#include <limits>
#include <cstddef>
#include <algorithm>
#include <functional>

// ========================= Math & FK =========================
using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Mat4f = Eigen::Matrix4f;

struct DHRow {
    double a;      // link length along x_i
    double alpha;  // twist about x_i [rad]
    double d;      // offset along z_i [in]
    double theta0; // constant theta offset [rad]
};

static inline std::array<DHRow,6> puma_dh(double S6_in) {
    const double DEG = M_PI/180.0;
    return std::array<DHRow,6>{{
        {  0.0,   +90.0*DEG,  0.0, 0.0 },  // i=1, φ1
        { 17.0,     0.0*DEG,  5.9, 0.0 },  // i=2, θ2
        {  0.8,   270.0*DEG,  0.0, 0.0 },  // i=3, θ3 (== -90°)
        {  0.0,   +90.0*DEG, 17.0, 0.0 },  // i=4, θ4
        {  0.0,   +90.0*DEG,  0.0, 0.0 },  // i=5, θ5
        {  0.0,     0.0*DEG, S6_in,0.0 },  // i=6, θ6
    }};
}

static inline Mat4 dh(double a, double alpha, double d, double theta) {
    const double ct = std::cos(theta),  st = std::sin(theta);
    const double ca = std::cos(alpha),  sa = std::sin(alpha);
    Mat4 T;
    T <<  ct, -st*ca,  st*sa, a*ct,
          st,  ct*ca, -ct*sa, a*st,
           0,     sa,     ca,    d,
           0,      0,      0,    1;
    return T;
}

struct FKOut {
    Mat4 T06;
    std::array<Mat4,6> T_F_i; // cumulative to each link frame
};

static inline double rad(double d){ return d*M_PI/180.0; }

FKOut forward_ge_all(double phi1,double th2,double th3,double th4,double th5,double th6,double S6=4.0) {
    const auto DH = puma_dh(S6);
    const double q[6] = { phi1, th2, th3, th4, th5, th6 };
    FKOut out; out.T_F_i.fill(Mat4::Identity());
    Mat4 T = Mat4::Identity();
    for (int i = 0; i < 6; ++i) {
        T = T * dh(DH[i].a, DH[i].alpha, DH[i].d, q[i] + DH[i].theta0);
        out.T_F_i[i] = T;
    }
    out.T06 = T;
    return out;
}

Vec3 vecmult_F_from_6(const Mat4& T_6toF, const Vec3& p6) {
    Eigen::Vector4d h; h << p6(0), p6(1), p6(2), 1.0;
    Eigen::Vector4d out = T_6toF * h;
    return out.head<3>();
}

// ========================= Minimal GL helpers =========================
static const char* kGLSL = "#version 130";
static GLuint gProg = 0, gVao = 0, gVbo = 0;

static GLuint compileShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok=0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if(!ok){ char log[2048]; glGetShaderInfoLog(s,2048,nullptr,log); std::cerr<<"Shader error:\n"<<log<<"\n"; }
    return s;
}

static void createSimpleLineProgram() {
    const char* vs =
        "#version 130\n"
        "uniform mat4 uMVP;\n"
        "in vec3 aPos;\n"
        "void main(){ gl_Position = uMVP * vec4(aPos,1.0); }\n";
    const char* fs =
        "#version 130\n"
        "uniform vec3 uColor;\n"
        "out vec4 FragColor;\n"
        "void main(){ FragColor = vec4(uColor,1.0); }\n";
    GLuint v = compileShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileShader(GL_FRAGMENT_SHADER, fs);
    gProg = glCreateProgram();
    glAttachShader(gProg,v); glAttachShader(gProg,f);
    glBindAttribLocation(gProg, 0, "aPos");
    glLinkProgram(gProg);
    glDeleteShader(v); glDeleteShader(f);

    glGenVertexArrays(1, &gVao);
    glGenBuffers(1, &gVbo);
}



// ========================= Minimal STL loader & Mesh =========================
// Supports binary STL (typical) and ASCII STL (fallback). No materials; flat color draw.

// Interleaved vertex {pos.xyz, normal.xyz}  <-- define BEFORE Mesh
struct Vtx { float px,py,pz, nx,ny,nz; };

struct Mesh {
    GLuint vao = 0, vbo = 0;
    GLsizei count = 0;                 // number of vertices
    std::vector<Vtx> cpu;              // CPU copy for picking
    bool valid() const { return vao != 0 && count > 0; }
};

static GLuint gMeshProg = 0; // shader for meshes (positions+normals)
// (no second Vtx here — it's already declared above)

// Normalize helper
static inline Eigen::Vector3f nrm(Eigen::Vector3f v){ float L=v.norm(); return (L>1e-12f)? v/L : v; }

// Try binary STL first; if header begins with "solid" fall back to ASCII parse
static bool loadSTL(const std::string& path, std::vector<Vtx>& out) {
    out.clear();
    std::ifstream f(path, std::ios::binary);
    if(!f) return false;

    // Peek first 5 chars
    char head5[6] = {0};
    f.read(head5, 5);
    f.seekg(0);
    bool asciiCandidate = (std::string(head5) == "solid");

    if (!asciiCandidate) {
        // Binary STL
        char header[80]; f.read(header, 80);
        uint32_t triCount=0; f.read(reinterpret_cast<char*>(&triCount), 4);
        if(!f || triCount==0) return false;
        out.reserve(triCount*3);
        for(uint32_t i=0;i<triCount;i++){
            float nx,ny,nz, vx[9]; uint16_t attr;
            f.read(reinterpret_cast<char*>(&nx),4);
            f.read(reinterpret_cast<char*>(&ny),4);
            f.read(reinterpret_cast<char*>(&nz),4);
            for(int k=0;k<9;k++) f.read(reinterpret_cast<char*>(&vx[k]),4);
            f.read(reinterpret_cast<char*>(&attr),2);
            // If normal is zero, recompute
            Eigen::Vector3f p0(vx[0],vx[1],vx[2]), p1(vx[3],vx[4],vx[5]), p2(vx[6],vx[7],vx[8]);
            Eigen::Vector3f N(nx,ny,nz);
            if (N.squaredNorm() < 1e-20f) N = nrm((p1-p0).cross(p2-p0));
            else N = nrm(N);
            Vtx a{p0.x(),p0.y(),p0.z(), N.x(),N.y(),N.z()};
            Vtx b{p1.x(),p1.y(),p1.z(), N.x(),N.y(),N.z()};
            Vtx c{p2.x(),p2.y(),p2.z(), N.x(),N.y(),N.z()};
            out.push_back(a); out.push_back(b); out.push_back(c);
        }
        return true;
    }

    // ASCII STL fallback (simple)
    std::string line;
    Eigen::Vector3f N(0,0,1);
    std::vector<Eigen::Vector3f> facet;
    facet.reserve(3);
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::string w; ss>>w;
        if (w=="facet") {
            std::string tmp; ss>>tmp; // "normal"
            float nx,ny,nz; ss>>nx>>ny>>nz; N = nrm(Eigen::Vector3f(nx,ny,nz));
        } else if (w=="vertex") {
            float x,y,z; ss>>x>>y>>z; facet.emplace_back(x,y,z);
            if (facet.size()==3) {
                if (N.squaredNorm()<1e-12f) N = nrm((facet[1]-facet[0]).cross(facet[2]-facet[0]));
                out.push_back({facet[0].x(),facet[0].y(),facet[0].z(), N.x(),N.y(),N.z()});
                out.push_back({facet[1].x(),facet[1].y(),facet[1].z(), N.x(),N.y(),N.z()});
                out.push_back({facet[2].x(),facet[2].y(),facet[2].z(), N.x(),N.y(),N.z()});
                facet.clear();
            }
        } else if (w=="endfacet") {
            facet.clear();
        }
    }
    return !out.empty();
}

static GLuint compileMeshShader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type); glShaderSource(s,1,&src,nullptr); glCompileShader(s);
    GLint ok=0; glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
    if(!ok){ char log[2048]; glGetShaderInfoLog(s,2048,nullptr,log); std::cerr<<"Mesh shader error:\n"<<log<<"\n"; }
    return s;
}

// ========================= URDF-ish local transforms =========================
static Mat4f rpy_xyz_to_mat4f(float rx,float ry,float rz, float tx,float ty,float tz) {
    // R = Rz * Ry * Rz? (URDF uses R = Rz(rz)*Ry(ry)*Rx(rx)), then translate
    float cx=std::cos(rx), sx=std::sin(rx);
    float cy=std::cos(ry), sy=std::sin(ry);
    float cz=std::cos(rz), sz=std::sin(rz);
    Eigen::Matrix3f Rx; Rx<<1,0,0, 0,cx,-sx, 0,sx,cx;
    Eigen::Matrix3f Ry; Ry<<cy,0,sy, 0,1,0, -sy,0,cy;
    Eigen::Matrix3f Rz; Rz<<cz,-sz,0, sz,cz,0, 0,0,1;
    Eigen::Matrix3f R = Rz*Ry*Rx;
    Mat4f M = Mat4f::Identity();
    M.block<3,3>(0,0) = R;
    M(0,3)=tx; M(1,3)=ty; M(2,3)=tz;
    return M;
}

// ADD: uniform scale matrix
static Mat4f scaleUniform(float s) {
    Mat4f S = Mat4f::Identity();
    S(0,0) = S(1,1) = S(2,2) = s;
    return S;
}

// URDF (Z-up) -> our world (Y-up): rotate -90° about X
static Mat4f buildUpAdapter(int signRx90, bool roll180, bool yaw180) {
    // Base: Rx(±90°) to map URDF Z-up into our Y-up
    float ax = signRx90 * (float(M_PI)/2.0f);
    float cx = std::cos(ax), sx = std::sin(ax);
    Mat4f Rx = Mat4f::Identity();
    Rx(1,1)=cx; Rx(1,2)=-sx;
    Rx(2,1)=sx; Rx(2,2)= cx;

    // Optional extra: roll 180° about X (flips “upside-down”)
    Mat4f Rx180 = Mat4f::Identity();
    Rx180(1,1) = -1.f; Rx180(2,2) = -1.f;

    // Optional extra: yaw 180° about Z (turn front/back around)
    Mat4f Rz180 = Mat4f::Identity();
    Rz180(0,0) = -1.f; Rz180(1,1) = -1.f;

    Mat4f M = Rx;
    if (roll180) M = Rx180 * M;
    if (yaw180)  M = Rz180 * M;
    return M;
}



static void createMeshProgram() {
    const char* vs =
        "#version 130\n"
        "uniform mat4 uMVP;\n"
        "uniform mat4 uModel;\n"
        "in vec3 aPos; in vec3 aNrm;\n"
        "out vec3 vN;\n"
        "void main(){\n"
        "  vN = mat3(uModel) * aNrm;\n"
        "  gl_Position = uMVP * uModel * vec4(aPos,1.0);\n"
        "}\n";
    const char* fs =
        "#version 130\n"
        "in vec3 vN; uniform vec3 uColor; out vec4 FragColor;\n"
        "void main(){\n"
        "  vec3 N = normalize(vN);\n"
        "  float d = max(dot(N, normalize(vec3(0.4,0.7,0.6))), 0.0);\n"
        "  float a = 0.25;\n"
        "  vec3 c = (a + d*0.75) * uColor;\n"
        "  FragColor = vec4(c,1.0);\n"
        "}\n";
    GLuint v = compileMeshShader(GL_VERTEX_SHADER, vs);
    GLuint f = compileMeshShader(GL_FRAGMENT_SHADER, fs);
    gMeshProg = glCreateProgram();
    glAttachShader(gMeshProg,v); glAttachShader(gMeshProg,f);
    glBindAttribLocation(gMeshProg, 0, "aPos");
    glBindAttribLocation(gMeshProg, 1, "aNrm");
    glLinkProgram(gMeshProg);
    glDeleteShader(v); glDeleteShader(f);
}

static Mesh uploadMesh(const std::vector<Vtx>& verts) {
    Mesh m;
    if(verts.empty()) return m;
    m.cpu = verts; // keep a CPU copy for picking

    glGenVertexArrays(1, &m.vao);
    glBindVertexArray(m.vao);
    glGenBuffers(1, &m.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, m.vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(verts.size()*sizeof(Vtx)), verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vtx),(void*)offsetof(Vtx,px));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vtx),(void*)offsetof(Vtx,nx));
    glBindVertexArray(0);
    m.count = (GLsizei)verts.size();
    return m;
}

// ----------------- Debug primitives (procedural) -----------------
static Mesh gDbgCube, gDbgCyl;

static std::vector<Vtx> makeUnitCubeVerts()
{
    // Unit cube centered at origin, edge = 1, faces axis-aligned.
    // Triangles with per-face normals.
    const float p = 0.5f;
    struct P{float x,y,z;};
    auto v = [&](P a, P b, P c, P n){ return std::array<Vtx,3>{
        Vtx{a.x,a.y,a.z, n.x,n.y,n.z},
        Vtx{b.x,b.y,b.z, n.x,n.y,n.z},
        Vtx{c.x,c.y,c.z, n.x,n.y,n.z}
    };};

    std::vector<Vtx> out; out.reserve(36);
    // +X
    { auto t1=v({+p,-p,-p},{+p,+p,-p},{+p,+p,+p},{+1,0,0});
      auto t2=v({+p,-p,-p},{+p,+p,+p},{+p,-p,+p},{+1,0,0});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    // -X
    { auto t1=v({-p,-p,+p},{-p,+p,+p},{-p,+p,-p},{-1,0,0});
      auto t2=v({-p,-p,+p},{-p,+p,-p},{-p,-p,-p},{-1,0,0});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    // +Y
    { auto t1=v({-p,+p,-p},{-p,+p,+p},{+p,+p,+p},{0,+1,0});
      auto t2=v({-p,+p,-p},{+p,+p,+p},{+p,+p,-p},{0,+1,0});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    // -Y
    { auto t1=v({-p,-p,+p},{-p,-p,-p},{+p,-p,-p},{0,-1,0});
      auto t2=v({-p,-p,+p},{+p,-p,-p},{+p,-p,+p},{0,-1,0});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    // +Z
    { auto t1=v({-p,-p,+p},{+p,-p,+p},{+p,+p,+p},{0,0,+1});
      auto t2=v({-p,-p,+p},{+p,+p,+p},{-p,+p,+p},{0,0,+1});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    // -Z
    { auto t1=v({+p,-p,-p},{-p,-p,-p},{-p,+p,-p},{0,0,-1});
      auto t2=v({+p,-p,-p},{-p,+p,-p},{+p,+p,-p},{0,0,-1});
      out.insert(out.end(), t1.begin(), t1.end()); out.insert(out.end(), t2.begin(), t2.end()); }
    return out;
}

static std::vector<Vtx> makeUnitCylinderVerts(int seg=32)
{
    // Cylinder of height 1 (from y=-0.5 to y=+0.5), radius 0.5, centered at origin.
    const float R = 0.5f, H = 1.0f, yh = 0.5f;
    std::vector<Vtx> out;
    out.reserve(seg*12); // rough

    auto addTri = [&](Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f N){
        out.push_back({A.x(),A.y(),A.z(), N.x(),N.y(),N.z()});
        out.push_back({B.x(),B.y(),B.z(), N.x(),N.y(),N.z()});
        out.push_back({C.x(),C.y(),C.z(), N.x(),N.y(),N.z()});
    };

    // caps (+Y and -Y)
    for (int i=0;i<seg;i++){
        float a0= (float)i * (2.f*(float)M_PI/seg);
        float a1= (float)(i+1) * (2.f*(float)M_PI/seg);
        Eigen::Vector3f c(0,+yh,0);
        Eigen::Vector3f p0(R*std::cos(a0), +yh, R*std::sin(a0));
        Eigen::Vector3f p1(R*std::cos(a1), +yh, R*std::sin(a1));
        // top cap (normal +Y)
        addTri(c, p1, p0, {0,1,0});

        c = Eigen::Vector3f(0,-yh,0);
        p0 = {R*std::cos(a0), -yh, R*std::sin(a0)};
        p1 = {R*std::cos(a1), -yh, R*std::sin(a1)};
        // bottom cap (normal -Y)
        addTri(c, p0, p1, {0,-1,0});
    }

    // side
    for (int i=0;i<seg;i++){
        float a0= (float)i * (2.f*(float)M_PI/seg);
        float a1= (float)(i+1) * (2.f*(float)M_PI/seg);
        Eigen::Vector3f p0(R*std::cos(a0), -yh, R*std::sin(a0));
        Eigen::Vector3f p1(R*std::cos(a1), -yh, R*std::sin(a1));
        Eigen::Vector3f q0(R*std::cos(a0), +yh, R*std::sin(a0));
        Eigen::Vector3f q1(R*std::cos(a1), +yh, R*std::sin(a1));
        // normals (outward)
        Eigen::Vector3f n0(std::cos(a0), 0, std::sin(a0));
        Eigen::Vector3f n1(std::cos(a1), 0, std::sin(a1));
        // two tris (p0,q0,q1) and (p0,q1,p1)
        out.push_back({p0.x(),p0.y(),p0.z(), n0.x(),n0.y(),n0.z()});
        out.push_back({q0.x(),q0.y(),q0.z(), n0.x(),n0.y(),n0.z()});
        out.push_back({q1.x(),q1.y(),q1.z(), n1.x(),n1.y(),n1.z()});
        out.push_back({p0.x(),p0.y(),p0.z(), n0.x(),n0.y(),n0.z()});
        out.push_back({q1.x(),q1.y(),q1.z(), n1.x(),n1.y(),n1.z()});
        out.push_back({p1.x(),p1.y(),p1.z(), n1.x(),n1.y(),n1.z()});
    }
    return out;
}

static void ensureDebugMeshes()
{
    if (!gDbgCube.valid()) {
        auto v = makeUnitCubeVerts();
        // shrink cube down by factor (e.g. 0.05 → 5% original size)
        for (auto &vx : v) {
            vx.px *= 0.05f;
            vx.py *= 0.05f;
            vx.pz *= 0.05f;
        }
        gDbgCube = uploadMesh(v);
    }
    if (!gDbgCyl.valid()) {
        auto v = makeUnitCylinderVerts(48); // smoother
        for (auto &vx : v) {
            vx.px *= 0.05f;
            vx.py *= 0.05f;
            vx.pz *= 0.05f;
        }
        gDbgCyl = uploadMesh(v);
    }

}


static void drawMesh(const Mesh& m, const Mat4f& VP, const Mat4f& model, const Eigen::Vector3f& color) {
    if(!m.valid()) return;
    glUseProgram(gMeshProg);
    GLint locMVP = glGetUniformLocation(gMeshProg,"uMVP");
    GLint locM   = glGetUniformLocation(gMeshProg,"uModel");
    GLint locC   = glGetUniformLocation(gMeshProg,"uColor");
    Mat4f MVP = VP * model;
    glUniformMatrix4fv(locMVP,1,GL_FALSE,MVP.data());
    glUniformMatrix4fv(locM,1,GL_FALSE,model.data());
    glUniform3fv(locC,1,color.data());
    glBindVertexArray(m.vao);
    glDrawArrays(GL_TRIANGLES, 0, m.count);
    glBindVertexArray(0);
    glUseProgram(0);
}

static void drawLines(const std::vector<Eigen::Vector3f>& pts, const Mat4f& mvp, const Eigen::Vector3f& color, GLenum mode=GL_LINES) {
    glUseProgram(gProg);
    GLint locM = glGetUniformLocation(gProg, "uMVP");
    GLint locC = glGetUniformLocation(gProg, "uColor");
    glUniformMatrix4fv(locM, 1, GL_FALSE, mvp.data());
    glUniform3fv(locC, 1, color.data());

    glBindVertexArray(gVao);
    glBindBuffer(GL_ARRAY_BUFFER, gVbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(pts.size()*3*sizeof(float)), pts.data(), GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);

    glDrawArrays(mode, 0, (GLsizei)pts.size());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glUseProgram(0);
}

// ---- [ADD] Draw picked triangle as wireframe (feedback) ----
static void drawTriWire(const Eigen::Vector3f& a,
                        const Eigen::Vector3f& b,
                        const Eigen::Vector3f& c,
                        const Mat4f& mvp,
                        const Eigen::Vector3f& col)
{
    std::vector<Eigen::Vector3f> e = {a,b, b,c, c,a};
    drawLines(e, mvp, col);
}

static inline Eigen::Vector3f xformPoint(const Mat4f& M, const Eigen::Vector3f& p) {
    Eigen::Vector4f h(p.x(),p.y(),p.z(),1.0f);
    Eigen::Vector4f o = M*h;
    return o.head<3>();
}


// ---- [ADD] Fetch one triangle (3 verts) from a CPU mesh by first-index ----
static inline void getTriVertsWorld(const Mesh& m, int triFirst,
                                    const Mat4f& model,
                                    Eigen::Vector3f& A,
                                    Eigen::Vector3f& B,
                                    Eigen::Vector3f& C)
{
    const Vtx& a = m.cpu[triFirst + 0];
    const Vtx& b = m.cpu[triFirst + 1];
    const Vtx& c = m.cpu[triFirst + 2];
    A = xformPoint(model, Eigen::Vector3f(a.px,a.py,a.pz));
    B = xformPoint(model, Eigen::Vector3f(b.px,b.py,b.pz));
    C = xformPoint(model, Eigen::Vector3f(c.px,c.py,c.pz));
}

// --- Build an orthonormal basis given axis 'u' (unit). Returns e1, e2, u. ---
static inline void makeONB_from_axis(const Eigen::Vector3f& u,
                                     Eigen::Vector3f& e1,
                                     Eigen::Vector3f& e2)
{
    // Pick a vector least aligned with u
    Eigen::Vector3f a = (std::fabs(u.x()) < 0.9f) ? Eigen::Vector3f::UnitX() : Eigen::Vector3f::UnitY();
    e1 = (a - u * a.dot(u)).normalized();
    e2 = u.cross(e1).normalized();
}

// --- Simple 2D Kåsa circle fit: solves x^2 + y^2 + A x + B y + C = 0 ---
static inline bool fitCircleKasa(const std::vector<Eigen::Vector2f>& pts,
                                 Eigen::Vector2f& center, float& radius)
{
    if (pts.size() < 6) return false; // need a few points

    // Build normal equations
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();

    for (const auto& p : pts) {
        float x = p.x(), y = p.y();
        float xx = x*x, yy = y*y, s = xx + yy;
        Eigen::Vector3f v(x, y, 1.0f);
        M += v * v.transpose();
        b += -s * v;
    }

    Eigen::Vector3f sol = M.colPivHouseholderQr().solve(b); // [A, B, C]
    float A = sol.x(), B = sol.y(), C = sol.z();

    center = Eigen::Vector2f(-A*0.5f, -B*0.5f);
    float R2 = center.squaredNorm() - C;
    radius = (R2 > 0.0f) ? std::sqrt(R2) : 0.0f;
    return std::isfinite(radius) && radius > 1e-6f;
}


// Approximate world-space bbox diag for a mesh (quick & robust)
static float meshBBoxDiagWorld(const Mesh& m, const Mat4f& model) {
    if (!m.valid() || m.cpu.empty()) return 10.0f;
    Eigen::Vector3f lo( std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity());
    Eigen::Vector3f hi(-std::numeric_limits<float>::infinity(),
                       -std::numeric_limits<float>::infinity(),
                       -std::numeric_limits<float>::infinity());
    // sample every k-th vertex to keep it cheap
    const int step = std::max(1, (int)m.cpu.size()/5000);
    for (int i=0;i<(int)m.cpu.size(); i+=step) {
        const Vtx& v = m.cpu[i];
        Eigen::Vector3f pw = xformPoint(model, Eigen::Vector3f(v.px,v.py,v.pz));
        lo = lo.cwiseMin(pw);
        hi = hi.cwiseMax(pw);
    }
    return (hi - lo).norm();
}

// Circle fit residual (RMS) for 2D points and (center,radius)
static float circleFitResidualRMS(const std::vector<Eigen::Vector2f>& pts,
                                  const Eigen::Vector2f& c, float r) {
    if (pts.empty()) return std::numeric_limits<float>::infinity();
    double s = 0.0;
    for (auto& p : pts) {
        double d = (p - c).norm() - r;
        s += d*d;
    }
    return (float)std::sqrt(s / (double)pts.size());
}


struct Ray { Eigen::Vector3f o, d; };

static Ray computePickRay(double mouseX, double mouseY, int w, int h,
                          const Mat4f& V, const Mat4f& P)
{
    float x =  2.0f * float(mouseX) / float(w) - 1.0f;
    float y =  1.0f - 2.0f * float(mouseY) / float(h);
    Mat4f invVP = (P * V).inverse();
    Eigen::Vector4f p0 = invVP * Eigen::Vector4f(x, y, -1.0f, 1.0f);
    Eigen::Vector4f p1 = invVP * Eigen::Vector4f(x, y,  1.0f, 1.0f);
    p0 /= p0.w(); p1 /= p1.w();
    Ray r; r.o = p0.head<3>(); r.d = (p1.head<3>() - r.o).normalized();
    return r;
}

// Möller–Trumbore (returns t if hit, else +inf)
static float rayTri(const Ray& r, const Eigen::Vector3f& a,
                    const Eigen::Vector3f& b, const Eigen::Vector3f& c)
{
    const float EPS = 1e-7f;
    Eigen::Vector3f ab = b - a, ac = c - a;
    Eigen::Vector3f p  = r.d.cross(ac);
    float det = ab.dot(p);
    if (std::fabs(det) < EPS) return std::numeric_limits<float>::infinity();
    float invDet = 1.0f / det;
    Eigen::Vector3f tvec = r.o - a;
    float u = tvec.dot(p) * invDet;
    if (u < 0.0f || u > 1.0f) return std::numeric_limits<float>::infinity();
    Eigen::Vector3f q = tvec.cross(ab);
    float v = r.d.dot(q) * invDet;
    if (v < 0.0f || u + v > 1.0f) return std::numeric_limits<float>::infinity();
    float t = ac.dot(q) * invDet;
    return (t > EPS) ? t : std::numeric_limits<float>::infinity();
}

// ---- [ADD] Ray-cast a single mesh in world; return nearest t & tri index ----
static bool tryPickMesh(const Mesh& m, const Mat4f& model, const Ray& r,
                        float& outT, int& outTriFirst,
                        Eigen::Vector3f& outCentroid, Eigen::Vector3f& outNormal)
{
    if (!m.valid() || m.cpu.empty()) return false;

    // Build inverse model to transform ray into model space (faster)
    Mat4f invM = model.inverse();

    // Transform ray into model space
    Eigen::Vector4f o4(r.o.x(), r.o.y(), r.o.z(), 1.0f);
    Eigen::Vector4f d4(r.d.x(), r.d.y(), r.d.z(), 0.0f);
    Eigen::Vector3f o = (invM * o4).head<3>();
    Eigen::Vector3f d = (invM * d4).head<3>();
    Ray local; local.o = o; local.d = d.normalized();

    float bestT = std::numeric_limits<float>::infinity();
    int   bestFirst = -1;
    Eigen::Vector3f bestC(0,0,0), bestN(0,1,0);

    // Iterate triangles
    for (int i = 0; i+2 < (int)m.cpu.size(); i += 3) {
        const Vtx& a = m.cpu[i+0];
        const Vtx& b = m.cpu[i+1];
        const Vtx& c = m.cpu[i+2];
        Eigen::Vector3f A(a.px,a.py,a.pz), B(b.px,b.py,b.pz), C(c.px,c.py,c.pz);
        float t = rayTri(local, A, B, C);
        if (t < bestT) {
            bestT = t;
            bestFirst = i;
            Eigen::Vector3f N(a.nx,a.ny,a.nz); // STL stores per-facet normal per-vertex
            bestN = nrm(N);
            Eigen::Vector3f Ccent = (A+B+C)/3.0f;
            // back to world for centroid & normal
            Eigen::Vector4f Cw = model * Eigen::Vector4f(Ccent.x(),Ccent.y(),Ccent.z(),1.0f);
            Eigen::Vector4f Nw = model * Eigen::Vector4f(bestN.x(),bestN.y(),bestN.z(),0.0f);
            bestC = Cw.head<3>();
            bestN = nrm(Nw.head<3>());
        }
    }

    if (bestFirst >= 0 && std::isfinite(bestT)) {
        outT = bestT; outTriFirst = bestFirst; outCentroid = bestC; outNormal = bestN;
        return true;
    }
    return false;
}

struct FacePick {
    int meshIndex = -999;   // -1 = base, 0..5 = link mesh index, -999 = none
    int triFirst  = -1;     // first vertex index of the picked triangle (multiple of 3)
    Eigen::Vector3f c_world = Eigen::Vector3f::Zero();    // triangle centroid (world)
    Eigen::Vector3f n_world = Eigen::Vector3f::UnitZ();   // triangle normal (world)

    // Cylinder / round-face inference (optional)
    bool isCircular = false;                                // true if we detected a round surface
    Eigen::Vector3f cyl_axis = Eigen::Vector3f::UnitZ();    // axis direction (unit, world)
    Eigen::Vector3f cyl_center = Eigen::Vector3f::Zero();   // center point on the circular face (world)
};

// Project Q (points centered at hit.c_world) onto the plane ⟂ axis.
// If rimOnly=true, keep only the outer ring (top ~30% largest radii)
// and fit a circle there (Kåsa). Returns centerW, radius, RMS.
static bool fitCircleProjected(const std::vector<Eigen::Vector3f>& Q,
                               const Eigen::Vector3f& axis,
                               const Eigen::Vector3f& originW,
                               bool rimOnly,
                               Eigen::Vector3f& centerW,
                               float& radius,
                               float& rms)
{
    if (Q.size() < 6) return false;

    Eigen::Vector3f e1,e2; makeONB_from_axis(axis.normalized(), e1, e2);

    std::vector<Eigen::Vector2f> pts2D; pts2D.reserve(Q.size());
    for (auto& q : Q) {
        Eigen::Vector3f v = q - axis * q.dot(axis); // drop axis component
        pts2D.emplace_back(v.dot(e1), v.dot(e2));
    }
    if (pts2D.size() < 6) return false;

    // Optional: keep rim subset
    std::vector<Eigen::Vector2f> used = pts2D;
    if (rimOnly) {
        std::vector<float> rad; rad.reserve(pts2D.size());
        for (auto& p : pts2D) rad.push_back(p.norm());

        // threshold at top ~30%
        std::vector<float> tmp = rad;
        const size_t keep = std::max<size_t>(6, (size_t)(0.30 * tmp.size()));
        std::nth_element(tmp.begin(), tmp.end() - keep, tmp.end());
        float thr = tmp[tmp.size() - keep];

        used.clear(); used.reserve(keep);
        for (size_t i=0; i<pts2D.size(); ++i)
            if (rad[i] >= thr) used.push_back(pts2D[i]);

        if (used.size() < 6) return false;
    }

    Eigen::Vector2f c2; float r=0.0f;
    if (!fitCircleKasa(used, c2, r)) return false;

    centerW = originW + e1*c2.x() + e2*c2.y();
    radius  = r;
    rms     = circleFitResidualRMS(used, c2, r);
    return std::isfinite(radius) && radius > 1e-4f;
}


// Infer cylinder/round face near a triangle hit by analyzing nearby normals for a common axis,
// then fitting a circle in the plane orthogonal to that axis to find the center.
static bool analyzeCircularNeighborhood(const Mesh& m,
                                        const Mat4f& model,
                                        const FacePick& hit,
                                        FacePick& outPick)
{
    if (!m.valid() || hit.triFirst < 0) return false;

    // Larger neighborhood so we can reach the rim even from the center.
    const float diag = meshBBoxDiagWorld(m, model);
    const float R = std::max(0.12f * diag, 12.0f);

    std::vector<Eigen::Vector3f> PtsW;  PtsW.reserve(800);
    std::vector<Eigen::Vector3f> NrmW;  NrmW.reserve(800);
    Eigen::Matrix3f Nrot = model.block<3,3>(0,0);

    // Collect a patch around the hit
    const int step = std::max(1, (int)m.cpu.size()/20000);
    for (int i = 0; i + 2 < (int)m.cpu.size(); i += 3*step) {
        const Vtx& a = m.cpu[i+0], &b = m.cpu[i+1], &c = m.cpu[i+2];
        Eigen::Vector3f Aw = xformPoint(model, {a.px,a.py,a.pz});
        Eigen::Vector3f Bw = xformPoint(model, {b.px,b.py,b.pz});
        Eigen::Vector3f Cw = xformPoint(model, {c.px,c.py,c.pz});
        Eigen::Vector3f centroid = (Aw + Bw + Cw) / 3.0f;
        if ((centroid - hit.c_world).norm() <= R) {
            PtsW.push_back(Aw); PtsW.push_back(Bw); PtsW.push_back(Cw);
            Eigen::Vector3f N(a.nx,a.ny,a.nz);
            N = (Nrot * N).normalized();
            NrmW.push_back(N);
        }
    }
    if (PtsW.size() < 24) return false;

    // Center positions at the hit for PCA
    std::vector<Eigen::Vector3f> Q; Q.reserve(PtsW.size());
    for (auto& p : PtsW) Q.push_back(p - hit.c_world);

    auto cov3 = [](const std::vector<Eigen::Vector3f>& X)->Eigen::Matrix3f{
        Eigen::Matrix3f C=Eigen::Matrix3f::Zero();
        for (auto& v : X) C += v * v.transpose();
        C /= std::max<size_t>(1, X.size());
        return C;
    };

    // Axis candidates
    Eigen::Matrix3f Cn = cov3(NrmW);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esN(Cn);
    Eigen::Vector3f axisA = (esN.info()==Eigen::Success) ? esN.eigenvectors().col(0).normalized() : hit.n_world; // side wall

    Eigen::Matrix3f Cp = cov3(Q);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esP(Cp);
    Eigen::Vector3f axisB = (esP.info()==Eigen::Success) ? esP.eigenvectors().col(0).normalized() : hit.n_world; // cap plane normal
    Eigen::Vector3f axisC = (esP.info()==Eigen::Success) ? esP.eigenvectors().col(2).normalized() : hit.n_world; // cylinder axis

    // Try circle fits; for cap (axisB) use rimOnly=true
    bool any=false; float bestRMS=std::numeric_limits<float>::infinity(), bestR=0.0f;
    Eigen::Vector3f bestC, bestAxis;

    auto consider = [&](const Eigen::Vector3f& ax, bool rimOnly){
        Eigen::Vector3f Cw; float Rfit, RMS;
        if (fitCircleProjected(Q, ax, hit.c_world, rimOnly, Cw, Rfit, RMS)) {
            if (RMS < bestRMS) { bestRMS=RMS; bestR=Rfit; bestC=Cw; bestAxis=ax.normalized(); any=true; }
        }
    };

    consider(axisA, false); // side wall from normals
    consider(axisC, false); // side wall from positions
    consider(axisB, true ); // **cap**: fit rim only

    if (!any) return false;

    // quality gates
    if (bestR <= 1e-4f) return false;
    if (bestRMS > 0.06f * bestR) return false; // a bit stricter since we used rim

    outPick.isCircular = true;
    outPick.cyl_axis   = bestAxis;
    outPick.cyl_center = bestC;
    return true;
}


static Mat4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center, const Eigen::Vector3f& up) {
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);
    Mat4f M = Mat4f::Identity();
    M(0,0)=s.x(); M(0,1)=s.y(); M(0,2)=s.z();
    M(1,0)=u.x(); M(1,1)=u.y(); M(1,2)=u.z();
    M(2,0)=-f.x();M(2,1)=-f.y();M(2,2)=-f.z();
    Mat4f T = Mat4f::Identity();
    T(0,3) = -eye.x(); T(1,3) = -eye.y(); T(2,3) = -eye.z();
    return M*T;
}

static Mat4f perspective(float fovy_rad, float aspect, float znear, float zfar) {
    float f = 1.0f/std::tan(fovy_rad/2.0f);
    Mat4f P = Mat4f::Zero();
    P(0,0)=f/aspect; P(1,1)=f; P(2,2)=(zfar+znear)/(znear-zfar); P(2,3)=(2*zfar*znear)/(znear-zfar); P(3,2)=-1.0f;
    return P;
}

static Mat4f ortho(float l,float r,float b,float t,float zn,float zf){
    Mat4f M = Mat4f::Identity();
    M(0,0)=2.0f/(r-l); M(1,1)=2.0f/(t-b); M(2,2)=-2.0f/(zf-zn);
    M(0,3)=-(r+l)/(r-l); M(1,3)=-(t+b)/(t-b); M(2,3)=-(zf+zn)/(zf-zn);
    return M;
}

// ========================= ImGui panel state =========================
struct PumaUI {
    float deg[6] = {225.f,150.f,-60.f,45.f,60.f,-30.f};
    float S6 = 4.0f;
    float tool6[3] = {5.f,3.f,7.f};
    bool drawCoords = true;
    bool perspective = true;
    bool drawMeshes = true;
    float meshScale = 1.0f;
    bool flipUp = false;
    bool roll180 = false;
    bool yaw180  = true;

    // Debug scene toggles (draw ONLY a primitive in the 3D view)
    bool dbgCube = false;
    bool dbgCyl  = false;

    // --- Joint tool UI state ---
    bool showDefineJoint = false;   // NEW: gate the popup
    bool pickStationary = false;
    bool pickMoving = false;
    bool rot180_X = false, rot180_Y = false, rot180_Z = false;
};



static void draw_puma_panel(PumaUI& ui) {
    ImGui::Begin("Puma - Forward Analysis", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::TextUnformatted("Input Items (angles are in degrees)");
    ImGui::SliderFloat("phi 1",   &ui.deg[0], -180.f, 180.f, "%.2f");
    ImGui::SliderFloat("theta 2", &ui.deg[1], -180.f, 180.f, "%.2f");
    ImGui::SliderFloat("theta 3", &ui.deg[2], -180.f, 180.f, "%.2f");
    ImGui::SliderFloat("theta 4", &ui.deg[3], -180.f, 180.f, "%.2f");
    ImGui::SliderFloat("theta 5", &ui.deg[4], -180.f, 180.f, "%.2f");
    ImGui::SliderFloat("theta 6", &ui.deg[5], -180.f, 180.f, "%.2f");

    ImGui::Separator();
    ImGui::TextUnformatted("Tool Point in 6th Coord. Sys. (in)");
    ImGui::InputFloat("X", &ui.tool6[0], 0.1f, 1.0f, "%.3f");
    ImGui::InputFloat("Y", &ui.tool6[1], 0.1f, 1.0f, "%.3f");
    ImGui::InputFloat("Z", &ui.tool6[2], 0.1f, 1.0f, "%.3f");

    ImGui::Checkbox("draw coords", &ui.drawCoords);
    ImGui::SameLine();
    ImGui::Checkbox("perspective", &ui.perspective);
    ImGui::Checkbox("flip Z-up adapter", &ui.flipUp);
    ImGui::SameLine();
    ImGui::Checkbox("roll 180°", &ui.roll180);
    ImGui::SameLine();
    ImGui::Checkbox("yaw 180°", &ui.yaw180);
    ImGui::Checkbox("draw meshes", &ui.drawMeshes);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.f);
    ImGui::InputFloat("mesh scale", &ui.meshScale, 0.1f, 1.0f, "%.5f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Scale applied to mesh units -> inches.\nExamples:\n 1.0  = meshes already inches\n 39.3701 = meters -> inches\n 0.03937 = millimeters -> inches");

    // --- Debug scene toggles ---
    ImGui::Separator();
    ImGui::Checkbox("Debug cube", &ui.dbgCube);
    ImGui::SameLine();
    ImGui::Checkbox("Debug cylinder", &ui.dbgCyl);
    if (ui.dbgCube && ui.dbgCyl) {
        // mutually exclusive: keep the latest click
        if (ImGui::IsItemEdited()) ui.dbgCube = false; // if cylinder was just edited
    }

    ImGui::Separator();


    // NEW: a button to open the modal tool
    if (ImGui::Button("Define Joint Tool...")) {
        ui.showDefineJoint = true;
        ImGui::OpenPopup("Define Joint");
    }

    ImGui::End();
}

// ----- simple string helpers -----
static bool readWholeFile(const std::string& path, std::string& outText) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss; ss << in.rdbuf();
    outText = ss.str();
    return true;
}

static bool extractAttrTriplet(const std::string& src, const std::string& key, float& x, float& y, float& z) {
    // find key="a b c"
    auto kpos = src.find(key + "=\"");
    if (kpos == std::string::npos) return false;
    kpos += key.size() + 2; // move past key="
    auto endq = src.find('"', kpos);
    if (endq == std::string::npos) return false;
    std::istringstream ss(src.substr(kpos, endq - kpos));
    return (ss >> x >> y >> z).good();
}

// Parse all <link name="..."><visual>...<origin xyz="..." rpy="..."/>
// Returns map: link_name -> Mat4f (Rz*Ry*Rx; then translation)
static std::unordered_map<std::string, Mat4f> parseURDFOrigins(const std::string& urdfPath) {
    std::unordered_map<std::string, Mat4f> out;
    std::string text;
    if (!readWholeFile(urdfPath, text)) return out;

    // Iterate over all <link ...> blocks
    const std::string tag = "<link";
    size_t pos = 0;
    while ((pos = text.find(tag, pos)) != std::string::npos) {
        // link name
        auto nameAttr = text.find("name=\"", pos);
        if (nameAttr == std::string::npos) { pos += tag.size(); continue; }
        nameAttr += 6;
        auto nameEnd = text.find('"', nameAttr);
        if (nameEnd == std::string::npos) break;
        std::string linkName = text.substr(nameAttr, nameEnd - nameAttr);

        // find end of link block
        auto linkEnd = text.find("</link>", nameEnd);
        if (linkEnd == std::string::npos) break;

        // search for <visual> ... <origin .../> in this block
        auto visualPos = text.find("<visual", nameEnd);
        if (visualPos == std::string::npos || visualPos > linkEnd) { pos = linkEnd; continue; }
        auto visualEnd = text.find("</visual>", visualPos);
        if (visualEnd == std::string::npos || visualEnd > linkEnd) visualEnd = linkEnd;

        // find the first <origin ... /> tag in visual block
        auto originPos = text.find("<origin", visualPos);
        if (originPos == std::string::npos || originPos > visualEnd) { pos = linkEnd; continue; }
        auto originEnd = text.find('>', originPos);
        if (originEnd == std::string::npos || originEnd > visualEnd) { pos = linkEnd; continue; }

        // slice containing the tag attributes
        std::string originTag = text.substr(originPos, originEnd - originPos + 1);

        float tx=0, ty=0, tz=0, rr=0, rp=0, ry=0;
        bool hasXYZ = extractAttrTriplet(originTag, "xyz", tx, ty, tz);
        bool hasRPY = extractAttrTriplet(originTag, "rpy", rr, rp, ry);
        if (!hasXYZ) { tx=ty=tz=0; }
        if (!hasRPY) { rr=rp=ry=0; }

        // URDF units: meters/radians typically. You can scale xyz to inches later.
        Mat4f M = rpy_xyz_to_mat4f(rr, rp, ry, tx, ty, tz);
        out[linkName] = M;
        pos = linkEnd;
    }
    return out;
}

struct UrdfJoint {
    std::string name;
    std::string parent, child; // link names
    float ox=0, oy=0, oz=0;    // joint origin xyz (meters)
    float rr=0, rp=0, ry=0;    // joint origin rpy (rad)
    float ax=1, ay=0, az=0;    // joint axis (unit; default x)
    bool revolute=true;        // assume revolute unless 'fixed'
};

// reuse readWholeFile(), extractAttrTriplet() from earlier

static bool extractAttrStr(const std::string& src, const std::string& key, std::string& out) {
    auto kpos = src.find(key + "=\""); if (kpos==std::string::npos) return false;
    kpos += key.size()+2; auto endq = src.find('"', kpos); if (endq==std::string::npos) return false;
    out = src.substr(kpos, endq-kpos); return true;
}

static std::vector<UrdfJoint> parseURDFJoints(const std::string& urdfPath) {
    std::vector<UrdfJoint> joints;
    std::string text; if(!readWholeFile(urdfPath, text)) return joints;
    size_t pos=0;
    while ((pos=text.find("<joint", pos)) != std::string::npos) {
        auto end = text.find("</joint>", pos); if (end==std::string::npos) break;
        std::string block = text.substr(pos, end-pos);
        UrdfJoint j;
        extractAttrStr(block, "name", j.name);
        std::string type; extractAttrStr(block, "type", type);
        j.revolute = (type!="fixed");

        // parent/child
        auto ppos = block.find("<parent"); if(ppos!=std::string::npos) {
            auto pend = block.find('>', ppos);
            std::string tag = block.substr(ppos, pend-ppos+1);
            extractAttrStr(tag, "link", j.parent);
        }
        auto cpos = block.find("<child"); if(cpos!=std::string::npos) {
            auto cend = block.find('>', cpos);
            std::string tag = block.substr(cpos, cend-cpos+1);
            extractAttrStr(tag, "link", j.child);
        }

        // origin
        auto opos = block.find("<origin");
        if (opos!=std::string::npos) {
            auto oend = block.find('>', opos);
            std::string tag = block.substr(opos, oend-opos+1);
            extractAttrTriplet(tag, "xyz", j.ox, j.oy, j.oz);
            extractAttrTriplet(tag, "rpy", j.rr, j.rp, j.ry);
        }
        // axis
        auto axpos = block.find("<axis");
        if (axpos!=std::string::npos) {
            auto axend = block.find('>', axpos);
            std::string tag = block.substr(axpos, axend-axpos+1);
            extractAttrTriplet(tag, "xyz", j.ax, j.ay, j.az);
            // normalize
            float L = std::sqrt(j.ax*j.ax + j.ay*j.ay + j.az*j.az);
            if (L>1e-8f){ j.ax/=L; j.ay/=L; j.az/=L; }
        }
        joints.push_back(j);
        pos = end + 8;
    }
    return joints;
}

// rotation about arbitrary axis
static Mat4f axisAngle(float ax,float ay,float az, float angle) {
    float c=std::cos(angle), s=std::sin(angle), t=1.f-c;
    Mat4f M = Mat4f::Identity();
    M(0,0)=t*ax*ax + c;     M(0,1)=t*ax*ay - s*az; M(0,2)=t*ax*az + s*ay;
    M(1,0)=t*ax*ay + s*az;  M(1,1)=t*ay*ay + c;    M(1,2)=t*ay*az - s*ax;
    M(2,0)=t*ax*az - s*ay;  M(2,1)=t*ay*az + s*ax; M(2,2)=t*az*az + c;
    return M;
}

// Build world->link transforms from URDF joints and the 6 slider angles.
// Required: jointNames[0..5] in the same order as your sliders, and a base link name.
struct URDFFK {
    std::unordered_map<std::string, Mat4f> T_world_link;
    std::array<Mat4f,6> T_world_link_i; // ordered for your 6 links
};

static URDFFK fk_from_urdf(const std::vector<UrdfJoint>& joints,
                           const std::string& baseLinkName,
                           const std::array<std::string,6>& linkNames,
                           const std::array<std::string,6>& jointNames,
                           const float qrad[6])
{
    // Build a quick lookup: child -> joint
    std::unordered_map<std::string, UrdfJoint> byChild;
    for (auto& j: joints) byChild[j.child] = j;

    URDFFK out;
    out.T_world_link.clear();

    // Set base link at identity (URDF frame)
    out.T_world_link[baseLinkName] = Mat4f::Identity();

    // Walk the chain by jointNames in order
    Mat4f T = Mat4f::Identity();
    std::string parent = baseLinkName;

    for (int i=0;i<6;++i) {
        // Find the joint by name: find child link that uses this joint name
        // (if you name joints -> easier: build a map name->joint)
        const UrdfJoint* pj = nullptr;
        for (auto& j: joints) {
            if (j.name == jointNames[i]) { pj = &j; break; }
        }
        if (!pj) { // fallback: try by child link name
            auto it = byChild.find(linkNames[i]);
            if (it == byChild.end()) { out.T_world_link_i[i]=T; continue; }
            pj = &it->second;
        }

        // parent->joint origin (meters, radians)
        Mat4f Tor = rpy_xyz_to_mat4f(pj->rr, pj->rp, pj->ry, pj->ox, pj->oy, pj->oz);
        Mat4f Rax = pj->revolute ? axisAngle(pj->ax, pj->ay, pj->az, qrad[i]) : Mat4f::Identity();

        // accumulate: parent link -> joint origin -> joint rotation -> child link
        T = T * Tor * Rax;

        // child link is now at T in URDF coords
        out.T_world_link[pj->child] = T;
        out.T_world_link_i[i] = T;

        parent = pj->child;
    }
    return out;
}

// ---- Auto-discover base link and 6-joint chain from URDF ----
struct UrdfChain {
    std::string baseLinkName;
    std::array<std::string,6> jointNames;
    std::array<std::string,6> linkNames; // children of those joints, in order
    bool ok = false;
};

static UrdfChain buildChainFromUrdf(const std::vector<UrdfJoint>& joints) {
    UrdfChain c; c.ok = false;
    if (joints.empty()) return c;

    // --- find base link (appears as parent but never as child)
    std::unordered_map<std::string,int> parentCount, childCount;
    for (auto& j : joints) { parentCount[j.parent]++; childCount[j.child]++; }
    std::string base;
    for (auto& kv : parentCount) {
        if (!childCount.count(kv.first)) { base = kv.first; break; }
    }
    if (base.empty()) return c;
    c.baseLinkName = base;

    // --- adjacency: parent link -> outgoing joints
    std::unordered_map<std::string, std::vector<const UrdfJoint*>> byParent;
    for (auto& j: joints) byParent[j.parent].push_back(&j);

    // Helper: given a link, depth-first search the longest path of revolute joints
    struct Path { std::vector<const UrdfJoint*> js; };
    std::function<Path(const std::string&)> dfs = [&](const std::string& link)->Path{
        Path best;
        auto it = byParent.find(link);
        if (it == byParent.end()) return best;
        // Try all outgoing revolute joints; pick the path with the most steps
        for (const UrdfJoint* pj : it->second) {
            if (!pj->revolute) continue;
            Path sub = dfs(pj->child);
            Path cur; cur.js.reserve(1 + sub.js.size());
            cur.js.push_back(pj);
            cur.js.insert(cur.js.end(), sub.js.begin(), sub.js.end());
            if (cur.js.size() > best.js.size()) best = std::move(cur);
        }
        return best;
    };

    Path longest = dfs(base);
    if (longest.js.size() < 6) {
        // still allow <6 if model is partial, but mark not-ok
        // (you will still see meshes; FK will be identity for missing)
        // Return best effort so UI can show what was found.
        for (size_t i=0; i<longest.js.size() && i<6; ++i) {
            c.jointNames[i] = longest.js[i]->name;
            c.linkNames[i]  = longest.js[i]->child;
        }
        return c; // c.ok remains false
    }

    // take the first 6 joints from the longest path
    for (int i=0; i<6; ++i) {
        c.jointNames[i] = longest.js[i]->name;
        c.linkNames[i]  = longest.js[i]->child;
    }
    c.ok = true;
    return c;
}


static void resetPick(FacePick& p){ p = FacePick(); }

// ========================= Main =========================
int main() {
    // --- GLFW / GL ---
    if (!glfwInit()) { std::cerr<<"GLFW init failed\n"; return 1; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,0);
    GLFWwindow* window = glfwCreateWindow(1280, 800, "Puma - Forward Analysis", nullptr, nullptr);
    if (!window) { std::cerr<<"Window create failed\n"; glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr<<"GLAD load failed\n"; return 1;
    }
    createSimpleLineProgram();

    createMeshProgram();

    // --- Load PUMA link meshes (set your real file paths here) ---
    // Base mesh (world-attached)
    std::string baseSTL =
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Base-with-revolute-v1_Base-with-revolute-on-it.stl";

    // Link meshes (one per joint 1..6)
    std::vector<std::string> linkSTL = {
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Middle-acting-revolute-to-revolute-v1_Middle-acting-revolute-to-revolute.stl",            // link 1
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Revolute-to-middle-acting-revolute-v1_Revolute-to-middle-acting-revolute.stl",        // link 2
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Right-angle-revolute-to-middle-acting-revolute-v1_right-angle-revolute-to-middle-acting-revolute.stl", // link 3
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Revolute-to-right-angle-revolute-v1_Revolute-to-right-angle-revolute.stl",            // link 4
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Right-Angle-middle-acting-revolute-to-revolute-v1_Right-angle-middle-acting-revolute-to-revolute.stl", // link 5
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Revolute-to-end-effector-v1_Revolute-to-end-effector.stl"                           // link 6 (forearm/EE carrier)
    };

    // Base & link locals (filled from URDF below)
    Mat4f T_baseLocal = Mat4f::Identity();
    std::array<Mat4f,6> T_linkLocal;
    for(int i=0;i<6;++i){ T_linkLocal[i] = Mat4f::Identity(); }

    // --- URDF mapping (auto-discovered from file) ---
    const std::string urdfPath =
        "C:/Users/sorak/Documents/UF/Spring 2025/MIL/Autonomous stabilized turret system/Simulations C++/assets/puma/Chapter.urdf";

    // Parse URDF for <visual><origin .../> (per-link locals) and <joint ...> (topology)
    auto urdfLocals = parseURDFOrigins(urdfPath);
    auto urdfJoints = parseURDFJoints(urdfPath);

    // Build the 6-joint serial chain automatically
    UrdfChain chain = buildChainFromUrdf(urdfJoints);

    // DEBUG: print discovered chain once
    if (chain.ok) {
        std::cerr << "[URDF chain]\n";
        std::cerr << "  base link: " << chain.baseLinkName << "\n";
        for (int i = 0; i < 6; ++i) {
            std::cerr << "  j" << (i+1) << ": " << chain.jointNames[i]
                      << "  -> link: " << chain.linkNames[i] << "\n";
        }
    } else {
        std::cerr << "[URDF chain] auto-discovery failed\n";
    }

    // Fallback: if auto failed, keep Identity (you'll still see meshes)
    std::string baseLinkName = chain.ok ? chain.baseLinkName : std::string();
    std::array<std::string,6> linkNames = chain.ok ? chain.linkNames
                                                   : std::array<std::string,6>{"","","","","",""};
    std::array<std::string,6> jointNames = chain.ok ? chain.jointNames
                                                    : std::array<std::string,6>{"","","","","",""};

    // Assign base local (if present)
    if (!baseLinkName.empty() && urdfLocals.count(baseLinkName)) {
        T_baseLocal = urdfLocals[baseLinkName];
    }

    // Assign each link i (if present)
    for (int i = 0; i < 6; ++i) {
        auto it = urdfLocals.find(linkNames[i]);
        if (it != urdfLocals.end()) {
            T_linkLocal[i] = it->second;
        }
    }


    // NOTE on units: URDF xyz is usually in meters. We keep them in meters here.
    // We apply the global scale (e.g., 39.3701 for m→in) at draw-time via scaleUniform(ui.meshScale).


    // Load base
    Mesh baseMesh;
    {
        std::vector<Vtx> verts;
        if(loadSTL(baseSTL, verts)) baseMesh = uploadMesh(verts);
        else std::cerr<<"Failed to load BASE STL: "<<baseSTL<<"\n";
    }
    // Load links
    std::array<Mesh,6> linkMesh;
    for(int i=0;i<6;++i){
        std::vector<Vtx> verts;
        if(loadSTL(linkSTL[i], verts)) linkMesh[i] = uploadMesh(verts);
        else std::cerr<<"Failed to load LINK STL: "<<linkSTL[i]<<"\n";
    }


    // --- ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsLight();
    ImGui_ImplGlfw_InitForOpenGL(window, true); // we'll install our own scroll callback
    ImGui_ImplOpenGL3_Init(kGLSL);

    // --- scroll wheel: accumulate, and also forward to ImGui
    static double gScrollAcc = 0.0;
    glfwSetScrollCallback(window, [](GLFWwindow* w, double xoff, double yoff){
        ImGui_ImplGlfw_ScrollCallback(w, xoff, yoff); // keep ImGui happy
        gScrollAcc += yoff;                            // use for camera zoom
    });

    // --- UI state & camera ---
    PumaUI ui;
    FacePick pickS, pickM;   // stationary, moving
    float camYaw = -0.9f, camPitch = -0.25f, camDist = 100.0f; // inches-scale view
    double lastX=0, lastY=0; bool rotating=false;

    // --- loop ---
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // simple mouse orbit (when not interacting with ImGui)
        ImGuiIO& io = ImGui::GetIO();
        // orbit only when not interacting with ImGui
        if (!io.WantCaptureMouse) {
            if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                if (!rotating) { glfwGetCursorPos(window,&lastX,&lastY); rotating=true; }
                double x,y; glfwGetCursorPos(window,&x,&y);
                camYaw   += float(x-lastX)*0.005f;
                camPitch += float(y-lastY)*0.005f;
                camPitch = std::max(-1.5f, std::min(1.5f, camPitch));
                lastX=x; lastY=y;
            } else rotating=false;
        }

        // --- ImGui frame ---
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        draw_puma_panel(ui);

        // --- after draw_puma_panel(ui); inside the frame, before rendering:
        if (ui.showDefineJoint && ImGui::BeginPopupModal("Define Joint", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            if (ImGui::Button(ui.pickStationary ? "Click a stationary face (ESC to cancel)" : "Select stationary face")) {
                ui.pickStationary = true; ui.pickMoving = false;
            }
            ImGui::SameLine();
            if (ImGui::Button(ui.pickMoving ? "Click a moving face (ESC to cancel)" : "Select moving face")) {
                ui.pickMoving = true; ui.pickStationary = false;
            }

            ImGui::Checkbox("flip X 180°", &ui.rot180_X); ImGui::SameLine();
            ImGui::Checkbox("flip Y 180°", &ui.rot180_Y); ImGui::SameLine();
            ImGui::Checkbox("flip Z 180°", &ui.rot180_Z);

            if (ImGui::Button("Close")) {
                ui.showDefineJoint = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // apply scroll that was accumulated by the GLFW callback this frame
        if (gScrollAcc != 0.0) {
            camDist *= (gScrollAcc > 0.0 ? 0.9f : 1.1f);
            camDist = std::max(1.0f, std::min(5000.0f, camDist));
            gScrollAcc = 0.0;
        }
        // keep scale sane
        if (ui.meshScale <= 0.0f) ui.meshScale = 1.0f;

        // URDF Z-up -> our Y-up; allow flip via UI
        Mat4f T_up = buildUpAdapter(ui.flipUp ? +1 : -1, ui.roll180, ui.yaw180);

        // --- compute FK & outputs ---
        auto fk = forward_ge_all(
            rad(ui.deg[0]), rad(ui.deg[1]), rad(ui.deg[2]),
            rad(ui.deg[3]), rad(ui.deg[4]), rad(ui.deg[5]),
            ui.S6
        );
        float qrad[6];
        for (int i = 0; i < 6; ++i)
            qrad[i] = static_cast<float>(rad(ui.deg[i]));

        URDFFK urdfFK = fk_from_urdf(urdfJoints, baseLinkName, linkNames, jointNames, qrad);

        Vec3 p6(ui.tool6[0], ui.tool6[1], ui.tool6[2]);
        Vec3 pF = vecmult_F_from_6(fk.T06, p6);
        Vec3 S6_F = fk.T06.block<3,1>(0,2);
        Vec3 a67_F = fk.T06.block<3,3>(0,0) * Vec3(1,0,0);

        ImGui::Begin("Output", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Tool Point in Fixed (in):  %.3f  %.3f  %.3f", pF.x(), pF.y(), pF.z());
        ImGui::Text("S6 Vector in Fixed:        %.4f  %.4f  %.4f", S6_F.x(), S6_F.y(), S6_F.z());
        ImGui::Text("a67 Vector in Fixed:       %.4f  %.4f  %.4f", a67_F.x(), a67_F.y(), a67_F.z());
        ImGui::End();

        // --- 3D render ---
        int w,h; glfwGetFramebufferSize(window,&w,&h);
        glViewport(0,0,w,h);
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.73f,0.85f,1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        // camera
        Eigen::Vector3f center(0, 20, 0); // look-at a bit above base
        Eigen::Vector3f eye(
            center.x() + camDist*std::cos(camPitch)*std::cos(camYaw),
            center.y() + camDist*std::sin(camPitch),
            center.z() + camDist*std::cos(camPitch)*std::sin(camYaw)
        );
        Mat4f V = lookAt(eye, center, Eigen::Vector3f(0,1,0));
        Mat4f P = ui.perspective ? perspective(45.0f*(float)M_PI/180.0f, (float)w/h, 1.0f, 5000.0f)
                                 : ortho(-150,150,-150,150,-2000,2000);
        Mat4f VP = P * V;

        // one switch for the whole frame:
        bool debugSceneActive = (ui.dbgCube || ui.dbgCyl);

        // === DEBUG SCENE OVERRIDE ===
        // Draw ONLY the debug primitive in the 3D view, but DO NOT early-out.
        if (debugSceneActive) {
            ensureDebugMeshes();

            const float dbgSize = 40.0f; // same as before; vertices already shrunk
            Mat4f dbgModel = scaleUniform(dbgSize);

            const Mesh& m = ui.dbgCube ? gDbgCube : gDbgCyl;
            drawMesh(m, VP, dbgModel, Eigen::Vector3f(0.75f,0.75f,0.78f));

            if (ui.drawCoords) {
                std::vector<Eigen::Vector3f> ax;
                float L=30.f;
                ax = { {0,0,0}, {L,0,0} }; drawLines(ax, VP, {1,0,0}); // X
                ax = { {0,0,0}, {0,L,0} }; drawLines(ax, VP, {0,1,0}); // Y
                ax = { {0,0,0}, {0,0,L} }; drawLines(ax, VP, {0,0,1}); // Z
            }
        }

        // If debug is active, *skip* robot/base/link meshes below (guard that section):
        // if (!debugSceneActive) { ... draw base & links ... }


        // ---- [ADD] Picking logic runs only while the modal is up ----
        // ---- [UNIFIED] Picking logic runs only while the modal is up ----
        // ---- Picking logic: always listen to the raw GLFW mouse (don’t gate on ImGui capture) ----
        if (ui.showDefineJoint) {
            // ESC cancels whichever pick mode is active
            if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                ui.pickStationary = false;
                ui.pickMoving     = false;
            }

            const bool wantPick = (ui.pickStationary || ui.pickMoving);

            // Edge-trigger on the left mouse using GLFW (works even while a modal is up)
            static bool prevDown = false;
            bool nowDown = wantPick && (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
            if (wantPick && nowDown && !prevDown) {
                // Mouse in WINDOW coords
                double mx, my; glfwGetCursorPos(window, &mx, &my);

                // Convert to FRAMEBUFFER coords (match what P uses)
                int ww, wh; glfwGetWindowSize(window, &ww, &wh);
                int fw, fh; glfwGetFramebufferSize(window, &fw, &fh);
                if (ww <= 0 || wh <= 0) { ww = 1; wh = 1; }
                if (fw <= 0 || fh <= 0) { fw = 1; fh = 1; }
                double mx_fb = mx * (double)fw / (double)ww;
                double my_fb = my * (double)fh / (double)wh;

                // Build the ray with the SAME V,P used for drawing
                Ray ray = computePickRay(mx_fb, my_fb, fw, fh, V, P);

                // Optional: debug the ray
                {
                    const float tNear = 1.0f, tFar = 2000.0f;
                    std::vector<Eigen::Vector3f> seg = { ray.o + ray.d*tNear, ray.o + ray.d*tFar };
                    drawLines(seg, VP, Eigen::Vector3f(0.1f,0.1f,0.1f));
                }

                // Nearest hit state
                float bestT = std::numeric_limits<float>::infinity();
                int   bestMeshIdx = -999;      // -2=debug prim, -1=base, 0..5=links
                int   bestFirst   = -1;
                Eigen::Vector3f bestC(0,0,0), bestN(0,1,0);

                // If the debug scene is active, ONLY test the debug primitive
                const bool debugSceneActive = (ui.dbgCube || ui.dbgCyl);
                if (debugSceneActive) {
                    const float dbgSize = 40.0f;
                    const Mesh& dm = ui.dbgCube ? gDbgCube : gDbgCyl;
                    Mat4f dbgModel = scaleUniform(dbgSize);
                    float t; int f; Eigen::Vector3f C,N;
                    if (tryPickMesh(dm, dbgModel, ray, t, f, C, N)) {
                        bestT = t; bestMeshIdx = -2; bestFirst = f; bestC = C; bestN = N;
                    }
                } else {
                    auto testOne = [&](int idx){
                        const Mesh& m = (idx == -1) ? baseMesh : linkMesh[idx];
                        if (!m.valid()) return;

                        Mat4f M;
                        if (idx == -1) {
                            M = (T_up * T_baseLocal);
                        } else {
                            M = (T_up * urdfFK.T_world_link_i[idx] * T_linkLocal[idx]);
                        }
                        M = M * scaleUniform(ui.meshScale);


                        float t; int f; Eigen::Vector3f C,N;
                        if (tryPickMesh(m, M, ray, t, f, C, N)) {
                            if (t < bestT) { bestT = t; bestMeshIdx = idx; bestFirst = f; bestC = C; bestN = N; }
                        }
                    };
                    testOne(-1);
                    for (int i=0;i<6;++i) testOne(i);
                }

                if (bestMeshIdx != -999) {
                    FacePick hit;
                    hit.meshIndex = bestMeshIdx;
                    hit.triFirst  = bestFirst;
                    hit.c_world   = bestC;
                    hit.n_world   = bestN;

                    // For circle/axis inference we need the same model used to draw
                    Mat4f M;
                    const Mesh* mm = nullptr;
                    if (bestMeshIdx == -2) {
                        const float dbgSize = 40.0f;
                        M  = scaleUniform(dbgSize);
                        mm = ui.dbgCube ? &gDbgCube : &gDbgCyl;
                    } else if (bestMeshIdx == -1) {
                        M  = T_up * T_baseLocal * scaleUniform(ui.meshScale);
                        mm = &baseMesh;
                    } else {
                        M  = T_up * urdfFK.T_world_link_i[bestMeshIdx] * T_linkLocal[bestMeshIdx] * scaleUniform(ui.meshScale);
                        mm = &linkMesh[bestMeshIdx];
                    }
                    analyzeCircularNeighborhood(*mm, M, hit, hit);

                    if (ui.pickStationary) { pickS = hit; ui.pickStationary = false; }
                    else if (ui.pickMoving) { pickM = hit; ui.pickMoving = false; }
                }
            }
            prevDown = nowDown;
        }


        // --- URDF chain debug polyline (magenta) ---
        {
            std::vector<Eigen::Vector3f> segsURDF;
            Mat4f T_up_now = T_up; // already computed this frame
            Eigen::Vector3f prev; bool havePrev = false;
            for (int i=0;i<6;++i) {
                Mat4f Ti = T_up_now * urdfFK.T_world_link_i[i] * scaleUniform(ui.meshScale);
                Eigen::Vector3f oi( Ti(0,3), Ti(1,3), Ti(2,3) );
                if (havePrev) { segsURDF.push_back(prev); segsURDF.push_back(oi); }
                prev = oi; havePrev = true;
            }
            drawLines(segsURDF, VP, Eigen::Vector3f(0.9f,0.1f,0.6f));
        }
        // tiny cross at picked centroids
        auto cross = [&](const Eigen::Vector3f& p, float L, const Eigen::Vector3f& col){
            std::vector<Eigen::Vector3f> s;
            s.push_back(p+Eigen::Vector3f(+L,0,0)); s.push_back(p+Eigen::Vector3f(-L,0,0));
            s.push_back(p+Eigen::Vector3f(0,+L,0)); s.push_back(p+Eigen::Vector3f(0,-L,0));
            s.push_back(p+Eigen::Vector3f(0,0,+L)); s.push_back(p+Eigen::Vector3f(0,0,-L));
            drawLines(s, VP, col);
        };
        if (pickS.meshIndex != -999) cross(pickS.c_world, 2.0f, Eigen::Vector3f(0,1,0)); // green
        if (pickM.meshIndex != -999) cross(pickM.c_world, 2.0f, Eigen::Vector3f(1,0,0)); // red
        auto drawPickedTri = [&](const FacePick& P, const Eigen::Vector3f& col){
            if (P.meshIndex == -999 || P.triFirst < 0) return;

            const Mesh* m = nullptr;
            Mat4f M;

            if (P.meshIndex == -2) {
                // Debug primitive
                const float dbgSize = 40.0f;
                m = ui.dbgCube ? &gDbgCube : &gDbgCyl;
                M = scaleUniform(dbgSize);
            } else if (P.meshIndex == -1) {
                m = &baseMesh;
                M = T_up * T_baseLocal * scaleUniform(ui.meshScale);
            } else {
                m = &linkMesh[P.meshIndex];
                M = T_up * urdfFK.T_world_link_i[P.meshIndex] * T_linkLocal[P.meshIndex] * scaleUniform(ui.meshScale);
            }

            Eigen::Vector3f A,B,C; getTriVertsWorld(*m, P.triFirst, M, A,B,C);
            drawTriWire(A,B,C, VP, col);
        };

        drawPickedTri(pickS, Eigen::Vector3f(0,1,0)); // green tri
        drawPickedTri(pickM, Eigen::Vector3f(1,0,0)); // red tri


        // If a circular face was recognized, draw its axis line through the center
        auto drawAxisThrough = [&](const FacePick& P, float halfLen, const Eigen::Vector3f& col){
            if (!P.isCircular) return;
            Eigen::Vector3f a = P.cyl_center - P.cyl_axis * halfLen;
            Eigen::Vector3f b = P.cyl_center + P.cyl_axis * halfLen;
            std::vector<Eigen::Vector3f> seg = {a,b};
            drawLines(seg, VP, col);
        };
        drawAxisThrough(pickS, 20.0f, Eigen::Vector3f(0,1,0)); // green
        drawAxisThrough(pickM, 20.0f, Eigen::Vector3f(1,0,0)); // red



        // draw base grid
        if (!debugSceneActive) {
            std::vector<Eigen::Vector3f> grid;
            const float s=100.f, step=10.f, y=0.f;
            for(float x=-s;x<=s;x+=step){ grid.push_back({x,y,-s}); grid.push_back({x,y,+s}); }
            for(float z=-s;z<=s;z+=step){ grid.push_back({-s,y,z}); grid.push_back({+s,y,z}); }
            drawLines(grid, VP, Eigen::Vector3f(0.85f,0.9f,1.0f));
        }


        // collect frame origins (inches)
        std::array<Eigen::Vector3f,7> O;
        O[0] = Eigen::Vector3f::Zero();
        for(int i=0;i<6;++i){
            O[i+1] = Eigen::Vector3f(
                float(fk.T_F_i[i](0,3)),
                float(fk.T_F_i[i](1,3)),
                float(fk.T_F_i[i](2,3))
            );
        }

        // links (orange/blue alternating)
        if (!debugSceneActive) {
            std::vector<Eigen::Vector3f> segs;
            for(int i=0;i<6;++i){ segs.push_back(O[i]); segs.push_back(O[i+1]); }
            drawLines(segs, VP, Eigen::Vector3f(1.0f,0.45f,0.15f));
        }


        // draw frame-6 axes if requested
        if (!debugSceneActive && ui.drawCoords) {
            Eigen::Matrix3f R6 = fk.T06.block<3,3>(0,0).cast<float>();
            Eigen::Vector3f o6 = O[6];
            float L = 10.f; // axis length
            std::vector<Eigen::Vector3f> axes;
            axes.push_back(o6); axes.push_back(o6 + R6.col(0)*L); // x (red)
            drawLines(axes, VP, Eigen::Vector3f(1,0,0)); axes.clear();
            axes.push_back(o6); axes.push_back(o6 + R6.col(1)*L); // y (green)
            drawLines(axes, VP, Eigen::Vector3f(0,0.8f,0)); axes.clear();
            axes.push_back(o6); axes.push_back(o6 + R6.col(2)*L); // z (blue)
            drawLines(axes, VP, Eigen::Vector3f(0,0,1));
        }

        // --- draw link meshes (if available) ---
        if (!debugSceneActive && ui.drawMeshes) {            // 1) draw BASE in world frame
            if (baseMesh.valid()) {
                Mat4f Mbase = T_up * T_baseLocal * scaleUniform(ui.meshScale);
                drawMesh(baseMesh, VP, Mbase, Eigen::Vector3f(0.35f,0.20f,0.10f));
            }
            // 2) draw each LINK in its joint frame with local URDF origin and global scale
            for(int i=0;i<6;++i){
                if (!linkMesh[i].valid()) continue;
                // URDF FK (meters) -> convert to our world (Y-up) and scale (-> inches)
                Mat4f T_urdf = urdfFK.T_world_link_i[i];
                Mat4f M = T_up * T_urdf * T_linkLocal[i] * scaleUniform(ui.meshScale);
                Eigen::Vector3f color = (i%2==0) ? Eigen::Vector3f(0.85f,0.45f,0.15f)
                                                 : Eigen::Vector3f(0.15f,0.45f,0.85f);
                drawMesh(linkMesh[i], VP, M, color);
            }
        }

        // ---- [ADD] Outline picked triangles (yellow) for feedback ----
        auto meshModelForIndex = [&](int idx)->Mat4f {
            if (idx == -1) {
                return T_up * T_baseLocal * scaleUniform(ui.meshScale);
            } else {
                Mat4f T_urdf = urdfFK.T_world_link_i[idx];
                return T_up * T_urdf * T_linkLocal[idx] * scaleUniform(ui.meshScale);
            }
        };

        auto drawPickOutline = [&](const FacePick& P){
            if (P.meshIndex == -999 || P.triFirst < 0) return;
            Eigen::Vector3f A,B,C;
            Mat4f M = meshModelForIndex(P.meshIndex);
            const Mesh& m = (P.meshIndex == -1) ? baseMesh : linkMesh[P.meshIndex];
            if (!m.valid()) return;
            getTriVertsWorld(m, P.triFirst, M, A,B,C);
            drawTriWire(A,B,C, VP, Eigen::Vector3f(1.0f, 1.0f, 0.0f));
        };
        drawPickOutline(pickS);
        drawPickOutline(pickM);


        // --- ImGui render ---
        ImGui::Render();
        // Make sure UI isn't depth-tested behind the 3D scene
        glDisable(GL_DEPTH_TEST);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

    }

    // cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
