#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>

#define WIDTH   1000
#define HEIGHT   750

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FOV_MIN     50.0f
#define FOV_MAX    900.0f
#define FOV_STEP    20.0f
#define FOV_DEFAULT 400.0f
/* camera is 10 units back so the full ring fits in view */
#define CAM_SCALE  (400.0f / 10.0f)

#define AUTO_PITCH  0.010f
#define AUTO_YAW    0.014f
#define AUTO_ROLL   0.007f
#define MANUAL_RATE 0.030f
#define DRAG_RATE   0.005f

#define MAX_CUBES   16
#define MIN_CUBES    1
#define NUM_DEFAULT  6
/* desired chord between cube centres; keeps ~1.5-unit gap between edges */
#define CUBE_CHORD   3.5f

typedef struct { float x, y, z; } Vec3;
typedef struct { float x, y; } Vec2;
typedef struct { float m[3][3]; } Mat3;

static Vec3 vertices[8] = {
    {-1,-1,-1},{ 1,-1,-1},{ 1, 1,-1},{-1, 1,-1},
    {-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},{-1, 1, 1},
};
static int edges[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7},
};

static Mat3 mat3_identity(void) {
    Mat3 r = {{{1,0,0},{0,1,0},{0,0,1}}};
    return r;
}

static Mat3 mat3_mul(Mat3 a, Mat3 b) {
    Mat3 r = {{{0}}};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
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
    Vec3 r0 = {m.m[0][0], m.m[0][1], m.m[0][2]};
    Vec3 r1 = {m.m[1][0], m.m[1][1], m.m[1][2]};
    float len, dot;
    len = sqrtf(r0.x*r0.x + r0.y*r0.y + r0.z*r0.z);
    r0.x /= len; r0.y /= len; r0.z /= len;
    dot = r1.x*r0.x + r1.y*r0.y + r1.z*r0.z;
    r1.x -= dot*r0.x; r1.y -= dot*r0.y; r1.z -= dot*r0.z;
    len = sqrtf(r1.x*r1.x + r1.y*r1.y + r1.z*r1.z);
    r1.x /= len; r1.y /= len; r1.z /= len;
    Vec3 r2 = { r0.y*r1.z - r0.z*r1.y,
                r0.z*r1.x - r0.x*r1.z,
                r0.x*r1.y - r0.y*r1.x };
    return (Mat3){{
        {r0.x, r0.y, r0.z},
        {r1.x, r1.y, r1.z},
        {r2.x, r2.y, r2.z},
    }};
}

static Mat3 rot_x(float a) {
    float c = cosf(a), s = sinf(a);
    return (Mat3){{  {1,0,0}, {0,c,-s}, {0,s,c}  }};
}
static Mat3 rot_y(float a) {
    float c = cosf(a), s = sinf(a);
    return (Mat3){{  {c,0,s}, {0,1,0}, {-s,0,c}  }};
}
static Mat3 rot_z(float a) {
    float c = cosf(a), s = sinf(a);
    return (Mat3){{  {c,-s,0}, {s,c,0}, {0,0,1}  }};
}

static Mat3 world_rotate(Mat3 orient, Mat3 r) {
    return mat3_mul(r, orient);
}

static Vec2 project(Vec3 v, float fov) {
    float cam_dist = fov / CAM_SCALE;
    float s = fov / (v.z + cam_dist);
    return (Vec2){ v.x*s + WIDTH/2.0f, -v.y*s + HEIGHT/2.0f };
}

static void draw_thick(SDL_Renderer *r, Vec2 a, Vec2 b) {
    SDL_RenderDrawLine(r, (int)a.x,   (int)a.y,   (int)b.x,   (int)b.y);
    SDL_RenderDrawLine(r, (int)a.x+1, (int)a.y,   (int)b.x+1, (int)b.y);
    SDL_RenderDrawLine(r, (int)a.x,   (int)a.y+1, (int)b.x,   (int)b.y+1);
}

/* ── tiny bitmap font (5×7, ASCII 32-127) ─────────────────────────────────── */
static const unsigned char font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x00,0x5F,0x00,0x00}, /* '!' */
    {0x00,0x07,0x00,0x07,0x00}, /* '"' */
    {0x14,0x7F,0x14,0x7F,0x14}, /* '#' */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* '$' */
    {0x23,0x13,0x08,0x64,0x62}, /* '%' */
    {0x36,0x49,0x55,0x22,0x50}, /* '&' */
    {0x00,0x05,0x03,0x00,0x00}, /* ''' */
    {0x00,0x1C,0x22,0x41,0x00}, /* '(' */
    {0x00,0x41,0x22,0x1C,0x00}, /* ')' */
    {0x14,0x08,0x3E,0x08,0x14}, /* '*' */
    {0x08,0x08,0x3E,0x08,0x08}, /* '+' */
    {0x00,0x50,0x30,0x00,0x00}, /* ',' */
    {0x08,0x08,0x08,0x08,0x08}, /* '-' */
    {0x00,0x60,0x60,0x00,0x00}, /* '.' */
    {0x20,0x10,0x08,0x04,0x02}, /* '/' */
    {0x3E,0x51,0x49,0x45,0x3E}, /* '0' */
    {0x00,0x42,0x7F,0x40,0x00}, /* '1' */
    {0x42,0x61,0x51,0x49,0x46}, /* '2' */
    {0x21,0x41,0x45,0x4B,0x31}, /* '3' */
    {0x18,0x14,0x12,0x7F,0x10}, /* '4' */
    {0x27,0x45,0x45,0x45,0x39}, /* '5' */
    {0x3C,0x4A,0x49,0x49,0x30}, /* '6' */
    {0x01,0x71,0x09,0x05,0x03}, /* '7' */
    {0x36,0x49,0x49,0x49,0x36}, /* '8' */
    {0x06,0x49,0x49,0x29,0x1E}, /* '9' */
    {0x00,0x36,0x36,0x00,0x00}, /* ':' */
    {0x00,0x56,0x36,0x00,0x00}, /* ';' */
    {0x08,0x14,0x22,0x41,0x00}, /* '<' */
    {0x14,0x14,0x14,0x14,0x14}, /* '=' */
    {0x00,0x41,0x22,0x14,0x08}, /* '>' */
    {0x02,0x01,0x51,0x09,0x06}, /* '?' */
    {0x32,0x49,0x79,0x41,0x3E}, /* '@' */
    {0x7E,0x11,0x11,0x11,0x7E}, /* 'A' */
    {0x7F,0x49,0x49,0x49,0x36}, /* 'B' */
    {0x3E,0x41,0x41,0x41,0x22}, /* 'C' */
    {0x7F,0x41,0x41,0x22,0x1C}, /* 'D' */
    {0x7F,0x49,0x49,0x49,0x41}, /* 'E' */
    {0x7F,0x09,0x09,0x09,0x01}, /* 'F' */
    {0x3E,0x41,0x49,0x49,0x7A}, /* 'G' */
    {0x7F,0x08,0x08,0x08,0x7F}, /* 'H' */
    {0x00,0x41,0x7F,0x41,0x00}, /* 'I' */
    {0x20,0x40,0x41,0x3F,0x01}, /* 'J' */
    {0x7F,0x08,0x14,0x22,0x41}, /* 'K' */
    {0x7F,0x40,0x40,0x40,0x40}, /* 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* 'M' */
    {0x7F,0x04,0x08,0x10,0x7F}, /* 'N' */
    {0x3E,0x41,0x41,0x41,0x3E}, /* 'O' */
    {0x7F,0x09,0x09,0x09,0x06}, /* 'P' */
    {0x3E,0x41,0x51,0x21,0x5E}, /* 'Q' */
    {0x7F,0x09,0x19,0x29,0x46}, /* 'R' */
    {0x46,0x49,0x49,0x49,0x31}, /* 'S' */
    {0x01,0x01,0x7F,0x01,0x01}, /* 'T' */
    {0x3F,0x40,0x40,0x40,0x3F}, /* 'U' */
    {0x1F,0x20,0x40,0x20,0x1F}, /* 'V' */
    {0x3F,0x40,0x38,0x40,0x3F}, /* 'W' */
    {0x63,0x14,0x08,0x14,0x63}, /* 'X' */
    {0x07,0x08,0x70,0x08,0x07}, /* 'Y' */
    {0x61,0x51,0x49,0x45,0x43}, /* 'Z' */
    {0x00,0x7F,0x41,0x41,0x00}, /* '[' */
    {0x02,0x04,0x08,0x10,0x20}, /* '\' */
    {0x00,0x41,0x41,0x7F,0x00}, /* ']' */
    {0x04,0x02,0x01,0x02,0x04}, /* '^' */
    {0x40,0x40,0x40,0x40,0x40}, /* '_' */
    {0x00,0x01,0x02,0x04,0x00}, /* '`' */
    {0x20,0x54,0x54,0x54,0x78}, /* 'a' */
    {0x7F,0x48,0x44,0x44,0x38}, /* 'b' */
    {0x38,0x44,0x44,0x44,0x20}, /* 'c' */
    {0x38,0x44,0x44,0x48,0x7F}, /* 'd' */
    {0x38,0x54,0x54,0x54,0x18}, /* 'e' */
    {0x08,0x7E,0x09,0x01,0x02}, /* 'f' */
    {0x0C,0x52,0x52,0x52,0x3E}, /* 'g' */
    {0x7F,0x08,0x04,0x04,0x78}, /* 'h' */
    {0x00,0x44,0x7D,0x40,0x00}, /* 'i' */
    {0x20,0x40,0x44,0x3D,0x00}, /* 'j' */
    {0x7F,0x10,0x28,0x44,0x00}, /* 'k' */
    {0x00,0x41,0x7F,0x40,0x00}, /* 'l' */
    {0x7C,0x04,0x18,0x04,0x78}, /* 'm' */
    {0x7C,0x08,0x04,0x04,0x78}, /* 'n' */
    {0x38,0x44,0x44,0x44,0x38}, /* 'o' */
    {0x7C,0x14,0x14,0x14,0x08}, /* 'p' */
    {0x08,0x14,0x14,0x18,0x7C}, /* 'q' */
    {0x7C,0x08,0x04,0x04,0x08}, /* 'r' */
    {0x48,0x54,0x54,0x54,0x20}, /* 's' */
    {0x04,0x3F,0x44,0x40,0x20}, /* 't' */
    {0x3C,0x40,0x40,0x20,0x7C}, /* 'u' */
    {0x1C,0x20,0x40,0x20,0x1C}, /* 'v' */
    {0x3C,0x40,0x30,0x40,0x3C}, /* 'w' */
    {0x44,0x28,0x10,0x28,0x44}, /* 'x' */
    {0x0C,0x50,0x50,0x50,0x3C}, /* 'y' */
    {0x44,0x64,0x54,0x4C,0x44}, /* 'z' */
    {0x00,0x08,0x36,0x41,0x00}, /* '{' */
    {0x00,0x00,0x7F,0x00,0x00}, /* '|' */
    {0x00,0x41,0x36,0x08,0x00}, /* '}' */
    {0x10,0x08,0x08,0x10,0x08}, /* '~' */
};

static void draw_char(SDL_Renderer *r, int cx, int cy, char c) {
    if (c < 32 || c > 126) return;
    const unsigned char *col = font5x7[(unsigned char)(c - 32)];
    for (int x = 0; x < 5; x++) {
        unsigned char bits = col[x];
        for (int y = 0; y < 7; y++) {
            if (bits & (1 << y))
                SDL_RenderDrawPoint(r, cx + x, cy + y);
        }
    }
}

static void draw_text(SDL_Renderer *r, int x, int y, const char *s) {
    for (; *s; s++, x += 6)
        draw_char(r, x, y, *s);
}

static void draw_hud(SDL_Renderer *r, int auto_spin, float fov, int num_cubes) {
    char buf[64];
    SDL_SetRenderDrawColor(r, 180, 180, 180, 255);

    draw_text(r, 10, 10,  "CONTROLS");
    draw_text(r, 10, 20,  "Up/Dn  : pitch");
    draw_text(r, 10, 29,  "Lf/Rt  : yaw");
    draw_text(r, 10, 38,  "Q / E  : roll");
    draw_text(r, 10, 47,  "+/-    : FOV");
    draw_text(r, 10, 56,  "[/]    : cubes");
    draw_text(r, 10, 65,  "SPACE  : auto-spin");
    draw_text(r, 10, 74,  "R      : reset");
    draw_text(r, 10, 83,  "Drag   : mouse look");

    SDL_SetRenderDrawColor(r, 100, 220, 120, 255);
    snprintf(buf, sizeof buf, "FOV  %4.0f", (double)fov);
    draw_text(r, 10, 99, buf);
    snprintf(buf, sizeof buf, "AUTO %s", auto_spin ? "ON " : "OFF");
    draw_text(r, 10, 108, buf);
    snprintf(buf, sizeof buf, "N    %d", num_cubes);
    draw_text(r, 10, 117, buf);
}

/* Radius of the ring so that chord between adjacent cube centres = CUBE_CHORD.
   N=1 returns 0 so the single cube sits at the origin. */
static float ring_radius(int n) {
    if (n <= 1) return 0.0f;
    return CUBE_CHORD / (2.0f * sinf((float)M_PI / n));
}

static void draw_cube(SDL_Renderer *ren, Mat3 orient, Vec3 centre, float fov) {
    Vec2 proj[8];
    for (int i = 0; i < 8; i++) {
        Vec3 v = mat3_apply(orient, vertices[i]);
        v.x += centre.x; v.y += centre.y; v.z += centre.z;
        proj[i] = project(v, fov);
    }
    SDL_SetRenderDrawColor(ren, 40, 120, 200, 255);
    for (int i = 0; i < 4; i++)
        draw_thick(ren, proj[edges[i][0]], proj[edges[i][1]]);
    SDL_SetRenderDrawColor(ren, 80, 180, 255, 255);
    for (int i = 8; i < 12; i++)
        draw_thick(ren, proj[edges[i][0]], proj[edges[i][1]]);
    SDL_SetRenderDrawColor(ren, 140, 220, 255, 255);
    for (int i = 4; i < 8; i++)
        draw_thick(ren, proj[edges[i][0]], proj[edges[i][1]]);
}

int main(void) {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window   *win = SDL_CreateWindow("Stonehenge Cubes",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    SDL_Renderer *ren = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    Mat3  orient     = world_rotate(world_rotate(mat3_identity(),
                           rot_x(0.3f)), rot_y(0.5f));
    float fov        = FOV_DEFAULT;
    int   auto_spin  = 1;
    int   dragging   = 0;
    int   drag_x = 0, drag_y = 0;
    int   frame      = 0;
    int   num_cubes  = NUM_DEFAULT;

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
                    orient     = mat3_identity();
                    fov        = FOV_DEFAULT;
                    auto_spin  = 1;
                    num_cubes  = NUM_DEFAULT;
                    break;
                case SDLK_EQUALS: case SDLK_PLUS:
                    fov = SDL_min(fov + FOV_STEP, FOV_MAX); break;
                case SDLK_MINUS:
                    fov = SDL_max(fov - FOV_STEP, FOV_MIN); break;
                case SDLK_LEFTBRACKET:
                    if (num_cubes > MIN_CUBES) num_cubes--;
                    break;
                case SDLK_RIGHTBRACKET:
                    if (num_cubes < MAX_CUBES) num_cubes++;
                    break;
                }
            }
            if (e.type == SDL_MOUSEWHEEL) {
                fov += e.wheel.y * FOV_STEP;
                fov = SDL_max(FOV_MIN, SDL_min(FOV_MAX, fov));
            }
            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                dragging = 1;
                drag_x = e.button.x;
                drag_y = e.button.y;
            }
            if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT)
                dragging = 0;
            if (e.type == SDL_MOUSEMOTION && dragging) {
                float dx = (e.motion.x - drag_x) * DRAG_RATE;
                float dy = (e.motion.y - drag_y) * DRAG_RATE;
                orient = world_rotate(orient, rot_y(-dx));
                orient = world_rotate(orient, rot_x(-dy));
                drag_x = e.motion.x;
                drag_y = e.motion.y;
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

        float r = ring_radius(num_cubes);
        for (int ci = 0; ci < num_cubes; ci++) {
            float angle = 2.0f * (float)M_PI * ci / num_cubes;
            Vec3 clocal = { r * cosf(angle), 0.0f, r * sinf(angle) };
            Vec3 cworld = mat3_apply(orient, clocal);
            /* rotate each cube so its face points toward the ring centre */
            Mat3 cube_orient = mat3_mul(orient, rot_y(-angle));
            draw_cube(ren, cube_orient, cworld, fov);
        }

        draw_hud(ren, auto_spin, fov, num_cubes);

        SDL_RenderPresent(ren);
    }

done:
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
