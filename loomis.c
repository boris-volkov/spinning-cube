#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>

static int SCR_W, SCR_H;   /* set at runtime from the actual display size */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FOV_MIN      50.0f
#define FOV_MAX     900.0f
#define FOV_STEP     20.0f
#define FOV_DEFAULT 380.0f
#define CAM_SCALE   (380.0f / 10.0f)

#define AUTO_PITCH  0.009f
#define AUTO_YAW    0.013f
#define AUTO_ROLL   0.006f
#define MANUAL_RATE 0.030f
#define DRAG_RATE   0.005f

/* ── Loomis head geometry ──────────────────────────────────────────────────
   Coordinate system: Y up, Z toward viewer (front of face), X right.
   Sphere = cranium.  Side cuts at x = ±X_CUT expose the flat temple planes.
   Face = half-cylinder (front/+Z half) hanging from brow down to chin.    */
#define HEAD_SCALE  4.50f   /* fallback; overridden at runtime from SCR_H   */
static float g_head_scale = HEAD_SCALE;
#define HEAD_R      1.00f   /* cranium sphere radius                        */
#define HEAD_X_CUT  0.70f   /* temple-cut x position (slightly wide)        */
#define FACE_R      HEAD_R                 /* same radius as sphere → face ring
                                              at y=0 is exactly the sphere's
                                              front semicircle; joins smoothly */
#define FACE_Z_OFF  0.0f                   /* cylinder axis through sphere centre */
#define FACE_Y_TOP  0.00f                  /* brow / top of face = equator   */
#define FACE_Y_BOT -1.40f                  /* chin                           */

#define SEGS   32           /* segments per arc (sphere/face rings)         */
#define MAX_EDGES 4000

typedef struct { float x, y, z; } Vec3;
typedef struct { float x, y;    } Vec2;
typedef struct { float m[3][3]; } Mat3;
typedef struct { Vec3 a, b;     } Edge3;

static Edge3 edges[MAX_EDGES];
static int   n_edges = 0;
static int   face_edge_start = 0;  /* set after sphere edges are generated */

/* ── math helpers (identical to cube.c) ───────────────────────────────── */
static Mat3 mat3_identity(void) {
    Mat3 r = {{{1,0,0},{0,1,0},{0,0,1}}}; return r;
}
static Mat3 mat3_mul(Mat3 a, Mat3 b) {
    Mat3 r = {{{0}}};
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k=0;k<3;k++)
        r.m[i][j] += a.m[i][k]*b.m[k][j];
    return r;
}
static Vec3 mat3_apply(Mat3 m, Vec3 v) {
    return (Vec3){
        m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z,
        m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z,
        m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z,
    };
}
static Mat3 mat3_orthonorm(Mat3 m) {
    Vec3 r0 = {m.m[0][0],m.m[0][1],m.m[0][2]};
    Vec3 r1 = {m.m[1][0],m.m[1][1],m.m[1][2]};
    float len,dot;
    len = sqrtf(r0.x*r0.x+r0.y*r0.y+r0.z*r0.z);
    r0.x/=len; r0.y/=len; r0.z/=len;
    dot = r1.x*r0.x+r1.y*r0.y+r1.z*r0.z;
    r1.x-=dot*r0.x; r1.y-=dot*r0.y; r1.z-=dot*r0.z;
    len = sqrtf(r1.x*r1.x+r1.y*r1.y+r1.z*r1.z);
    r1.x/=len; r1.y/=len; r1.z/=len;
    Vec3 r2 = { r0.y*r1.z-r0.z*r1.y, r0.z*r1.x-r0.x*r1.z, r0.x*r1.y-r0.y*r1.x };
    return (Mat3){{ {r0.x,r0.y,r0.z},{r1.x,r1.y,r1.z},{r2.x,r2.y,r2.z} }};
}
static Mat3 rot_x(float a) { float c=cosf(a),s=sinf(a); return (Mat3){{{1,0,0},{0,c,-s},{0,s,c}}}; }
static Mat3 rot_y(float a) { float c=cosf(a),s=sinf(a); return (Mat3){{{c,0,s},{0,1,0},{-s,0,c}}}; }
static Mat3 rot_z(float a) { float c=cosf(a),s=sinf(a); return (Mat3){{{c,-s,0},{s,c,0},{0,0,1}}}; }
static Mat3 world_rotate(Mat3 orient, Mat3 r) { return mat3_mul(r, orient); }

static Vec2 project(Vec3 v, float fov) {
    float cam_dist = fov / CAM_SCALE;
    float s = fov / (v.z + cam_dist);
    return (Vec2){ v.x*s + SCR_W*0.5f, -v.y*s + SCR_H*0.5f };
}
static void draw_thick(SDL_Renderer *r, Vec2 a, Vec2 b, int thick) {
    SDL_RenderDrawLine(r,(int)a.x,  (int)a.y,  (int)b.x,  (int)b.y);
    if (thick >= 2) {
        SDL_RenderDrawLine(r,(int)a.x+1,(int)a.y,  (int)b.x+1,(int)b.y);
        SDL_RenderDrawLine(r,(int)a.x,  (int)a.y+1,(int)b.x,  (int)b.y+1);
    }
    if (thick >= 3) {
        SDL_RenderDrawLine(r,(int)a.x-1,(int)a.y,  (int)b.x-1,(int)b.y);
        SDL_RenderDrawLine(r,(int)a.x,  (int)a.y-1,(int)b.x,  (int)b.y-1);
    }
}

/* ── tiny bitmap font (5×7) ───────────────────────────────────────────── */
static const unsigned char font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},
    {0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},{0x00,0x7F,0x41,0x41,0x00},
    {0x02,0x04,0x08,0x10,0x20},{0x00,0x41,0x41,0x7F,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},{0x00,0x01,0x02,0x04,0x00},{0x20,0x54,0x54,0x54,0x78},
    {0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},
    {0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},
    {0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},
    {0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},
    {0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},{0x7C,0x14,0x14,0x14,0x08},
    {0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},
    {0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},
    {0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},
    {0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},
    {0x00,0x41,0x36,0x08,0x00},{0x10,0x08,0x08,0x10,0x08},
};
static void draw_char(SDL_Renderer *r, int cx, int cy, char c) {
    if (c<32||c>126) return;
    const unsigned char *col = font5x7[(unsigned char)(c-32)];
    for (int x=0;x<5;x++) { unsigned char bits=col[x];
        for (int y=0;y<7;y++) if (bits&(1<<y)) SDL_RenderDrawPoint(r,cx+x,cy+y); }
}
static void draw_text(SDL_Renderer *r, int x, int y, const char *s) {
    for (;*s;s++,x+=6) draw_char(r,x,y,*s);
}
static void draw_hud(SDL_Renderer *r, int auto_spin, float fov) {
    char buf[64];
    SDL_SetRenderDrawColor(r,180,180,180,255);
    draw_text(r,10,10,"CONTROLS");
    draw_text(r,10,20,"Up/Dn : pitch");
    draw_text(r,10,29,"Lf/Rt : yaw");
    draw_text(r,10,38,"Q / E : roll");
    draw_text(r,10,47,"+/-   : FOV");
    draw_text(r,10,56,"SPACE : auto-spin");
    draw_text(r,10,65,"R     : reset");
    draw_text(r,10,74,"Drag  : mouse look");
    SDL_SetRenderDrawColor(r,100,220,120,255);
    snprintf(buf,sizeof buf,"FOV  %4.0f",(double)fov);
    draw_text(r,10,90,buf);
    snprintf(buf,sizeof buf,"AUTO %s",auto_spin?"ON ":"OFF");
    draw_text(r,10,99,buf);
}

/* ── edge accumulator ─────────────────────────────────────────────────── */
static void add_edge(Vec3 a, Vec3 b) {
    if (n_edges < MAX_EDGES) {
        a.x *= g_head_scale; a.y *= g_head_scale; a.z *= g_head_scale;
        b.x *= g_head_scale; b.y *= g_head_scale; b.z *= g_head_scale;
        edges[n_edges++] = (Edge3){a, b};
    }
}

/* Clip segment (a,b) to the slab |x| ≤ HEAD_X_CUT, then add it.
   When one endpoint is outside, the segment is linearly interpolated to
   land exactly on the cut plane — no gap, no slop constant needed.        */
static void add_edge_xclip(Vec3 a, Vec3 b) {
    float lim = HEAD_X_CUT;
    /* Discard if both outside on the same side */
    if (a.x >  lim && b.x >  lim) return;
    if (a.x < -lim && b.x < -lim) return;
    /* Clip against x = +lim */
    if (a.x > lim) {
        float t = (lim - b.x) / (a.x - b.x);
        a = (Vec3){ lim, b.y + t*(a.y-b.y), b.z + t*(a.z-b.z) };
    } else if (b.x > lim) {
        float t = (lim - a.x) / (b.x - a.x);
        b = (Vec3){ lim, a.y + t*(b.y-a.y), a.z + t*(b.z-a.z) };
    }
    /* Clip against x = -lim */
    if (a.x < -lim) {
        float t = (-lim - b.x) / (a.x - b.x);
        a = (Vec3){ -lim, b.y + t*(a.y-b.y), b.z + t*(a.z-b.z) };
    } else if (b.x < -lim) {
        float t = (-lim - a.x) / (b.x - a.x);
        b = (Vec3){ -lim, a.y + t*(b.y-a.y), a.z + t*(b.z-a.z) };
    }
    add_edge(a, b);
}

/* True for sphere surface points hidden behind the face half-cylinder.
   Strictly inside the cylinder's y range so the brow and chin rings show.  */
static int cyl_hidden(Vec3 p) {
    return p.z > 1e-4f && p.y < FACE_Y_TOP && p.y > FACE_Y_BOT;
}

/* x-clip then cylinder-occlusion clip.  If both endpoints are hidden, drop
   the segment.  If one is hidden, clip to the first boundary exited:
   z = 0, y = FACE_Y_TOP, or y = FACE_Y_BOT.                               */
static void add_edge_xclip_cyl(Vec3 a, Vec3 b) {
    int ha = cyl_hidden(a), hb = cyl_hidden(b);
    if (ha && hb) return;
    if (ha || hb) {
        Vec3 h = ha ? a : b;
        Vec3 v = ha ? b : a;
        float best_t = 1.0f;
        if (v.z <= 0.0f && h.z > 1e-6f)
            best_t = fminf(best_t, h.z / (h.z - v.z));
        if (v.y >= FACE_Y_TOP && h.y < FACE_Y_TOP - 1e-6f)
            best_t = fminf(best_t, (FACE_Y_TOP - h.y) / (v.y - h.y));
        if (v.y <= FACE_Y_BOT && h.y > FACE_Y_BOT + 1e-6f)
            best_t = fminf(best_t, (FACE_Y_BOT - h.y) / (v.y - h.y));
        Vec3 clip = { h.x + best_t*(v.x - h.x),
                      h.y + best_t*(v.y - h.y),
                      h.z + best_t*(v.z - h.z) };
        if (ha) a = clip; else b = clip;
    }
    add_edge_xclip(a, b);
}

/* Sphere latitude ring at height y, x-clipped and cylinder-occluded.      */
static void sphere_lat_ring(float y) {
    float r2 = HEAD_R*HEAD_R - y*y;
    if (r2 < 1e-4f) return;
    float r = sqrtf(r2);
    for (int i = 0; i < SEGS; i++) {
        float t0 = 2.0f*(float)M_PI*i/SEGS;
        float t1 = 2.0f*(float)M_PI*(i+1)/SEGS;
        Vec3 a = { r*sinf(t0), y, r*cosf(t0) };
        Vec3 b = { r*sinf(t1), y, r*cosf(t1) };
        add_edge_xclip_cyl(a, b);
    }
}

/* Sphere meridian (longitude arc) at azimuth phi, x-clipped and occluded. */
static void sphere_meridian(float phi) {
    for (int i = 0; i < SEGS; i++) {
        float l0 = -(float)M_PI/2 + (float)M_PI*i/SEGS;
        float l1 = -(float)M_PI/2 + (float)M_PI*(i+1)/SEGS;
        Vec3 a = { HEAD_R*cosf(l0)*sinf(phi), HEAD_R*sinf(l0), HEAD_R*cosf(l0)*cosf(phi) };
        Vec3 b = { HEAD_R*cosf(l1)*sinf(phi), HEAD_R*sinf(l1), HEAD_R*cosf(l1)*cosf(phi) };
        add_edge_xclip_cyl(a, b);
    }
}

/* Circle at x = ±X_CUT in the YZ plane (the flat temple cut face).       */
static void temple_cut_circle(float xc) {
    float rc = sqrtf(HEAD_R*HEAD_R - HEAD_X_CUT*HEAD_X_CUT);
    for (int i = 0; i < SEGS; i++) {
        float a0 = 2.0f*(float)M_PI*i/SEGS;
        float a1 = 2.0f*(float)M_PI*(i+1)/SEGS;
        Vec3 a = { xc, rc*cosf(a0), rc*sinf(a0) };
        Vec3 b = { xc, rc*cosf(a1), rc*sinf(a1) };
        add_edge(a, b);
    }
}

/* Face half-cylinder ring at height y, x-clipped to ±HEAD_X_CUT.         */
static void face_ring(float y) {
    for (int i = 0; i < SEGS; i++) {
        float t0 = (float)M_PI*i/SEGS;
        float t1 = (float)M_PI*(i+1)/SEGS;
        Vec3 a = { FACE_R*cosf(t0), y, FACE_Z_OFF + FACE_R*sinf(t0) };
        Vec3 b = { FACE_R*cosf(t1), y, FACE_Z_OFF + FACE_R*sinf(t1) };
        add_edge_xclip(a, b);
    }
}

/* Vertical stripe on the face cylinder at azimuth t, clipped to ±X_CUT.  */
static void face_vertical(float t) {
    float x = FACE_R*cosf(t), z = FACE_Z_OFF + FACE_R*sinf(t);
    if (fabsf(x) > HEAD_X_CUT + 0.01f) return;
    int N = 8;
    float dy = (FACE_Y_BOT - FACE_Y_TOP) / N;
    for (int i = 0; i < N; i++) {
        Vec3 a = { x, FACE_Y_TOP + i*dy,     z };
        Vec3 b = { x, FACE_Y_TOP + (i+1)*dy, z };
        add_edge(a, b);
    }
}

/* Clean cut-edge on the face cylinder at x = ±xc (matches the temple cut
   plane), running the full height of the face.                            */
static void face_cut_edge(float xc) {
    float z = sqrtf(fmaxf(0.0f, FACE_R*FACE_R - xc*xc));
    int N = 8;
    float dy = (FACE_Y_BOT - FACE_Y_TOP) / N;
    for (int i = 0; i < N; i++) {
        Vec3 a = { xc, FACE_Y_TOP + i*dy,     z };
        Vec3 b = { xc, FACE_Y_TOP + (i+1)*dy, z };
        add_edge(a, b);
    }
}

/* ── build the full edge list once at startup ─────────────────────────── */
static void generate_edges(void) {
    n_edges = 0;

    /* Cranium sphere – latitude rings */
    float lats[] = { -0.95f, -0.85f, -0.55f, -0.20f, 0.50f, 0.80f };
    for (int i = 0; i < (int)(sizeof lats/sizeof lats[0]); i++)
        sphere_lat_ring(lats[i]);

    /* Cranium sphere – meridians every 30° (12 total)                     */
    for (int i = 0; i < 12; i++)
        sphere_meridian(2.0f*(float)M_PI*i/12);

    /* Temple cut circles */
    temple_cut_circle( HEAD_X_CUT);
    temple_cut_circle(-HEAD_X_CUT);

    face_edge_start = n_edges;  /* record actual sphere edge count */

    /* Face half-cylinder – horizontal rings, evenly spaced 0 → -1.40      */
    float face_ys[] = { FACE_Y_TOP, -0.35f, -0.70f, -1.05f, FACE_Y_BOT };
    for (int i = 0; i < (int)(sizeof face_ys/sizeof face_ys[0]); i++)
        face_ring(face_ys[i]);

    /* Face half-cylinder – vertical lines (7 stripes including edges)     */
    for (int i = 0; i < 7; i++)
        face_vertical((float)M_PI*i/6);

    /* Cut-edge verticals on the face cylinder (front edge of jaw panel).   */
    face_cut_edge( HEAD_X_CUT);
    face_cut_edge(-HEAD_X_CUT);

    /* Jaw side panels: flat rectangles in the plane x = ±X_CUT that bridge
       from the face cylinder's cut edge (z = z_face) back to z = 0 (the
       centre plane of the sphere / temple circle centre).                  */
    float zf = sqrtf(fmaxf(0.0f, FACE_R*FACE_R - HEAD_X_CUT*HEAD_X_CUT));
    for (int side = -1; side <= 1; side += 2) {
        float xc = side * HEAD_X_CUT;

        /* Horizontal rails at each ring level */
        for (int i = 0; i < (int)(sizeof face_ys/sizeof face_ys[0]); i++) {
            add_edge((Vec3){xc, face_ys[i], zf},
                     (Vec3){xc, face_ys[i], 0.0f});
        }

        /* Closing vertical at z = 0 (back edge of jaw panel) */
        int N = 8;
        float dy = (FACE_Y_BOT - FACE_Y_TOP) / N;
        for (int i = 0; i < N; i++) {
            Vec3 a = { xc, FACE_Y_TOP + i*dy,     0.0f };
            Vec3 b = { xc, FACE_Y_TOP + (i+1)*dy, 0.0f };
            add_edge(a, b);
        }
    }

    /* Jawbone cross-bars: only the bottom two ring levels (below the sphere) */
    int nfy = (int)(sizeof face_ys / sizeof face_ys[0]);
    for (int i = nfy - 2; i < nfy; i++) {
        add_edge((Vec3){-HEAD_X_CUT, face_ys[i], 0.0f},
                 (Vec3){ HEAD_X_CUT, face_ys[i], 0.0f});
    }
}

/* ── draw the head with a given world-orientation matrix ─────────────── */
typedef struct { float mid_z; int ei; } DrawOrder;
static DrawOrder draw_order[MAX_EDGES];
static Vec3      draw_wa[MAX_EDGES];
static Vec3      draw_wb[MAX_EDGES];

static int cmp_draw_order(const void *a, const void *b) {
    float za = ((const DrawOrder *)a)->mid_z;
    float zb = ((const DrawOrder *)b)->mid_z;
    /* descending: farthest (largest z) drawn first */
    return (za > zb) ? -1 : (za < zb) ? 1 : 0;
}

static void draw_head(SDL_Renderer *ren, Mat3 orient, float fov) {
    float z_span = g_head_scale;

    for (int ei = 0; ei < n_edges; ei++) {
        draw_wa[ei] = mat3_apply(orient, edges[ei].a);
        draw_wb[ei] = mat3_apply(orient, edges[ei].b);
        draw_order[ei].mid_z = (draw_wa[ei].z + draw_wb[ei].z) * 0.5f;
        draw_order[ei].ei    = ei;
    }
    qsort(draw_order, n_edges, sizeof draw_order[0], cmp_draw_order);

    for (int di = 0; di < n_edges; di++) {
        int ei = draw_order[di].ei;
        Vec3 wa = draw_wa[ei];
        Vec3 wb = draw_wb[ei];

        /* depth_t: 0 = closest to viewer, 1 = furthest away */
        float mid_z   = draw_order[di].mid_z;
        float depth_t = (mid_z + z_span) / (2.0f * z_span);
        if (depth_t < 0.0f) depth_t = 0.0f;
        if (depth_t > 1.0f) depth_t = 1.0f;

        /* brightness: full intensity near, fades to ~25% at the back */
        float bright = 1.0f - depth_t * 0.75f;

        /* base colour: blue for cranium, gold for face/jaw */
        float br, bg, bb;
        if (ei < face_edge_start) {
            br = 60; bg = 130; bb = 210;
        } else {
            br = 220; bg = 160; bb = 80;
        }
        SDL_SetRenderDrawColor(ren,
            (Uint8)(br * bright),
            (Uint8)(bg * bright),
            (Uint8)(bb * bright),
            255);

        /* thickness: 3 px near, 2 px far */
        int thick = depth_t < 0.50f ? 3 : 2;

        Vec2 pa = project(wa, fov);
        Vec2 pb = project(wb, fov);
        draw_thick(ren, pa, pb, thick);
    }
}

/* ── main ─────────────────────────────────────────────────────────────── */
int main(int argc, char **argv) {
    const char *screenshot = NULL;
    for (int i = 1; i < argc; i++)
        if (argv[i][0] != '-' || argv[i][1] == 0) screenshot = argv[i];
        else if (i+1 < argc && (argv[i][1]=='s' || argv[i][1]=='o'))
            screenshot = argv[++i];

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    /* SDL_GetWindowSize lies on Wayland (returns 1×1 for fullscreen desktop).
       Query the display mode before creating the window — that's always right.
       Then tell the renderer to use those dimensions as the logical canvas so
       rendering coordinates match regardless of Wayland compositor scaling.  */
    int headless = SDL_strcmp(SDL_GetCurrentVideoDriver(), "offscreen") == 0;

    if (headless) {
        SCR_W = 1000; SCR_H = 750;
    } else {
        SDL_DisplayMode dm;
        if (SDL_GetCurrentDisplayMode(0, &dm) == 0) {
            SCR_W = dm.w; SCR_H = dm.h;
        } else {
            SCR_W = 1920; SCR_H = 1080;
        }
    }
    fprintf(stderr, "display: %d x %d\n", SCR_W, SCR_H);

    Uint32 win_flags = headless ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP;
    int init_w = headless ? SCR_W : 0;
    int init_h = headless ? SCR_H : 0;
    SDL_Window   *win = SDL_CreateWindow("Loomis Head",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, init_w, init_h, win_flags);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!headless) SDL_ShowCursor(SDL_DISABLE);

    SDL_RenderSetLogicalSize(ren, SCR_W, SCR_H);

    /* Scale head so it spans ~half the screen height.
       At z=0 the projection multiplier is CAM_SCALE (= FOV/cam_dist = 38).
       Model height from top of sphere (HEAD_R) to chin (FACE_Y_BOT) = 2.40. */
    g_head_scale = (SCR_H * 0.375f) / ((HEAD_R - FACE_Y_BOT) * CAM_SCALE);

    generate_edges();

    Mat3  orient    = world_rotate(world_rotate(mat3_identity(),
                          rot_x(0.25f)), rot_y((float)M_PI + 0.55f));
    float fov       = FOV_DEFAULT;
    int   auto_spin = 0;
    int   dragging  = 0, drag_x = 0, drag_y = 0;
    int   frame     = 0;
    const Uint8 *keys = SDL_GetKeyboardState(NULL);

    for (;;) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) goto done;
            if (e.type == SDL_KEYDOWN) {
                switch (e.key.keysym.sym) {
                case SDLK_ESCAPE: goto done;
                case SDLK_SPACE:  auto_spin = !auto_spin; break;
                case SDLK_r:
                    orient    = world_rotate(world_rotate(mat3_identity(),
                                    rot_x(0.25f)), rot_y((float)M_PI + 0.55f));
                    fov       = FOV_DEFAULT;
                    auto_spin = 0;
                    break;
                case SDLK_EQUALS: case SDLK_PLUS:
                    fov = SDL_min(fov+FOV_STEP, FOV_MAX); break;
                case SDLK_MINUS:
                    fov = SDL_max(fov-FOV_STEP, FOV_MIN); break;
                }
            }
            if (e.type == SDL_MOUSEWHEEL) {
                fov += e.wheel.y * FOV_STEP;
                fov  = SDL_max(FOV_MIN, SDL_min(FOV_MAX, fov));
            }
            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = 1; drag_x = e.button.x; drag_y = e.button.y;
            }
            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
                dragging = 0;
            if (e.type == SDL_MOUSEMOTION && dragging) {
                float dx = (e.motion.x - drag_x) * DRAG_RATE;
                float dy = (e.motion.y - drag_y) * DRAG_RATE;
                orient = world_rotate(orient, rot_y(-dx));
                orient = world_rotate(orient, rot_x(-dy));
                drag_x = e.motion.x; drag_y = e.motion.y;
            }
        }

        if (keys[SDL_SCANCODE_UP])    orient = mat3_mul(orient, rot_x(-MANUAL_RATE));
        if (keys[SDL_SCANCODE_DOWN])  orient = mat3_mul(orient, rot_x( MANUAL_RATE));
        if (keys[SDL_SCANCODE_LEFT])  orient = mat3_mul(orient, rot_y(-MANUAL_RATE));
        if (keys[SDL_SCANCODE_RIGHT]) orient = mat3_mul(orient, rot_y( MANUAL_RATE));
        if (keys[SDL_SCANCODE_Q])     orient = mat3_mul(orient, rot_z(-MANUAL_RATE));
        if (keys[SDL_SCANCODE_E])     orient = mat3_mul(orient, rot_z( MANUAL_RATE));

        if (auto_spin) {
            orient = mat3_mul(orient, rot_x(AUTO_PITCH));
            orient = mat3_mul(orient, rot_y(AUTO_YAW));
            orient = mat3_mul(orient, rot_z(AUTO_ROLL));
        }
        if (++frame % 120 == 0)
            orient = mat3_orthonorm(orient);

        SDL_SetRenderDrawColor(ren, 10, 10, 20, 255);
        SDL_RenderClear(ren);

        draw_head(ren, orient, fov);
        draw_hud(ren, auto_spin, fov);

        SDL_RenderPresent(ren);

        if (screenshot) {
            SDL_Surface *surf = SDL_CreateRGBSurfaceWithFormat(
                0, SCR_W, SCR_H, 32, SDL_PIXELFORMAT_ARGB8888);
            SDL_RenderReadPixels(ren, NULL, SDL_PIXELFORMAT_ARGB8888,
                                 surf->pixels, surf->pitch);
            SDL_SaveBMP(surf, screenshot);
            SDL_FreeSurface(surf);
            goto done;
        }
    }
done:
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
