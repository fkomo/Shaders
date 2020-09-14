// max ray marching steps
#define MAX_STEPS 128
// max ray marching distance
#define MAX_DISTANCE 100.0
// min ray marching distance
#define EPSILON 0.001
// pi
#define PI 3.141592

// ---- basic math operations -------------------------------------------------------------------------------------------------

// 2d rotation
mat2 rotate(in float angle)
{
    float s = sin(angle);
    float c = cos(angle);
    return mat2(c, -s, s, c);
}

// 3d translation
vec3 translate(in vec3 p, in vec3 v)
{
    return p - v;
}

// ---- boolean operations ----------------------------------------------------------------------------------------------------

// boolean not
float sd_difference(in float ta, in float tb)
{
    return max(ta, -tb);
}

// boolean and
float sd_intersection(in float ta, in float tb)
{
    return max(ta, tb);
}

// boolean or
float sd_union(in float ta, in float tb)
{
    return min(ta, tb);
}

// smooth boolean or
float sd_union_smooth(in float ta, in float tb, float strength)
{
    // TODO smooth union
    return sd_union(ta, tb);
}

// ---- objects ---------------------------------------------------------------------------------------------------------------

// plane
float sd_plane(in vec3 p, in vec3 n)
{
    return dot(p, normalize(n));
}

// used to round hard edges
float sd_round_edge(in float p, in float rounding_factor)
{
    return p - rounding_factor;
}

// sphere
float sd_sphere(in vec3 p, in float radius)
{
	return length(p) - radius;
}

// torus in xz plane
float sd_torus_xz(in vec3 p, in float radius1, in float radius2)
{
    float x = length(p.xz) - radius1;
    return length(vec2(x, p.y)) - radius2;
}

// line segment between 2 points sdf
float sd_line(in vec3 p, in vec3 a, in vec3 b, in float radius)
{
    float t = min(1.0, max(0.0, dot(p - a, b - a) / dot(b - a, b - a)));
    return length(p - a - (b - a) * t) - radius;
}

// upward line segment from origin to (0, height, 0)
float sd_line(in vec3 p, in float height, in float radius)
{
    p.y -= min(height, max(0.0, p.y)); // elongation operation
    return sd_sphere(p, radius);
}

// cylinder
float sd_cylinder(in vec3 p, in vec3 a, in vec3 b, in float radius)
{
    vec3 ab = b - a;
    vec3 ap = p - a;
    float t = dot(ab, ap) / dot(ab, ab);

    float x = length(p - (a + t * ab)) - radius;
    float y = length(ab) * (abs(t - 0.5) - 0.5);
    float e = length(max(vec2(x, y), 0.0));
    float i = min(max(x, y), 0.0);

    return e + i;
}

// upward cylinder (capped line) from origin to (0, height, 0)
float sd_cylinder(in vec3 p, in float height, in float radius)
{
    float t = p.y / height;
    p.y -= height * t;

    float x = length(p) - radius;
    float y = height * (abs(t - 0.5) - 0.5);

    return length(max(vec2(x, y), 0.0)) + min(max(x, y), 0.0);
}

// axis aligned box
float sd_box(in vec3 p, in vec3 size)
{
    vec3 q = abs(p) - size;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

// ---- ray marching ----------------------------------------------------------------------------------------------------------

// distance from p to scene
vec4 map(in vec3 p, in float time)
{
    float t = 0.0;

    vec3 p_trans = p;
    t = sd_sphere(p_trans, 0.2);

	p_trans = p;
    p_trans = translate(p, vec3(-0.6, 0.0, 0.0));
    t = sd_union(t, sd_box(p_trans, vec3(0.15, 0.3, 0.1)));

	p_trans = translate(p, vec3(-0.4, -0.05, 0.5));
    t = sd_union(t, sd_round_edge(sd_box(p_trans, vec3(0.3, 0.1, 0.2)), 0.01));

	p_trans = p;
    p_trans = translate(p, vec3(-0.3, -0.1, -0.5));
    t = sd_union(t, sd_line(p_trans, 0.4, 0.1));

	p_trans = p;
    p_trans = translate(p, vec3(0.8, -0.1, 0.5));
    t = sd_union(t, sd_cylinder(p_trans, 0.1, 0.2));

	p_trans = p;
    p_trans = translate(p, vec3(0.3, -0.1, 0.5));
    t = sd_union(t, sd_round_edge(sd_cylinder(p_trans, 0.4, 0.1), 0.01));

	p_trans = p;
    p_trans = translate(p, vec3(0.5, 0.1, 0.0));
    p_trans.xy *= rotate(PI * 0.5);
    t = sd_union(t, sd_torus_xz(p_trans, 0.2, 0.05));

	p_trans = p;
    p_trans = translate(p, vec3(0.0, -0.2, 0.0));
    t = sd_union(t, sd_plane(p_trans, vec3(0.0, 1.0, 0.0)));

    return vec4(p, t);
}

// normal of point on surface (gradient)
vec3 normal(in vec3 p, in float time)
{
    vec2 e = vec2(EPSILON, 0.0);
    return normalize(vec3(
        map(p + e.xyy, time).w - map(p - e.xyy, time).w,
        map(p + e.yxy, time).w - map(p - e.yxy, time).w,
        map(p + e.yyx, time).w - map(p - e.yyx, time).w
    ));                         
}

// compute hard (raytraced) shadow (0.0 or 1.0)
float shadow(in vec3 origin, in vec3 dir, float min_t, float max_t, in float time)
{
    for (float t = min_t; t < max_t;)
    {
        float h = map(origin + t * dir, time).w;
        if (abs(h) < EPSILON)
            return 0.0;

        t += h;
    }

    return 1.0;
}

// compute soft shadow [0.0, 1.0]
float soft_shadow(in vec3 origin, in vec3 dir, float min_t, float max_t, in float k, in float time)
{
    float res = 1.0;
    float ph = 1e20;
    for (float t = min_t; t < max_t;)
    {
        float h = map(origin + t * dir, time).w;
        if (abs(h) < EPSILON)
            return 0.0;
            
        float y = h * h/ (2.0 * ph);
        float d = sqrt(h * h - y * y);
        res = min(res, k * d / max(0.0, t - y));
        ph = h;
        t += h;
    }
    return res;
}

// compute ambient oclusion at specified point
float ambient_oclusion(in vec3 p, in vec3 n, in float time)
{
	float result = 0.0;
    float sca = 1.0;
    
    for (int i = 0; i < 5; i++)
    {
        float h = 0.01 + 0.11 * float(i) / 4.0;
        vec3 opos = p + h * n;
        float d = map(opos, time).w;
        
        result += (h - d) * sca;
        sca *= 0.95;
    }

    return clamp(1.0 - 2.0 * result, 0.0, 1.0);
}

// cast ray into scene
vec4 cast_ray(in vec3 origin, in vec3 dir, in float time)
{
    vec4 result = vec4(-1);
    
    float t = 0.0;
    for (int i = 0; i < MAX_STEPS && t < MAX_DISTANCE; ++i)
    {
        // map to scene
        vec4 d = map(origin + t * dir, time);
        
        // if inside
        if (abs(d.w) < EPSILON)
        {
            result = vec4(d.xyz, t);
            break;
        }
        
        t += d.w;
	}
    
    return result;
}

// calculate final pixel color
vec3 render(in vec2 output_pixel, in vec3 camera_origin, in mat3 camera, in float time)
{
    vec3 color = vec3(0.0);

    vec3 ray_dir = camera * normalize(vec3(output_pixel, 2.0));
    vec4 march = cast_ray(camera_origin, ray_dir, time);
    if (march.w > 0.0)
    {
        // scene hit
        vec3 n = normal(march.xyz, iTime);
        vec3 p = march.xyz + n * EPSILON;

        // scene object material        
        vec3 material = vec3(0.2);// * n;
        
        // sun direction
        vec3 sun_dir = normalize(vec3(0.8, 0.4, 0.2));
        // sun diffuse light
        float sun_diff = clamp(dot(n, sun_dir), 0.0, 1.0);
        // sky diffuse light
        float sky_diff = clamp(0.5 + 0.5 * dot(n, vec3(0.0, 1.0, 0.0)), 0.0, 1.0);
        // ambient diffuse light from bottom direction
  		float ambient_diff = clamp(0.5 + 0.5 * dot(n, vec3(0.0, -1.0, 0.0)), 0.0, 1.0);       
       
        // final material color with
        color = material * vec3(7.0, 5.0, 3.0) * sun_diff;
        color *= soft_shadow(p, sun_dir, EPSILON, MAX_DISTANCE, 10.0, time);
        color += material * vec3(0.5, 0.8, 0.9) * sky_diff;
        color += material * vec3(0.7, 0.3, 0.2) * ambient_diff;
        color *= ambient_oclusion(march.xyz, n, time);
    }
    else
    {
        // sky
        color = vec3(0.4, 0.7, 1.0) - 0.8 * ray_dir.y;
	    color = mix(color, vec3(0.7, 0.7, 0.8), exp(-10.0 * ray_dir.y));
	}
    
    return pow(color, vec3(0.4545)); // gamma correction
}

// ---- camera ----------------------------------------------------------------------------------------------------------------

// create camera matrix
mat3 camera(in vec3 camera_origin, in vec3 camera_target, float cr)
{
	vec3 cw = normalize(camera_target - camera_origin);
	vec3 cu = normalize(cross(cw, vec3(sin(cr), cos(cr), 0.0)));
	vec3 cv = cross(cu, cw);

    return mat3(cu, cv, cw);
}

// ---- main ------------------------------------------------------------------------------------------------------------------

// shader entry point
// uniform vec3      iResolution;           // viewport resolution (in pixels)
// uniform float     iTime;                 // shader playback time (in seconds)
// uniform float     iTimeDelta;            // render time (in seconds)
// uniform int       iFrame;                // shader playback frame
// uniform float     iChannelTime[4];       // channel playback time (in seconds)
// uniform vec3      iChannelResolution[4]; // channel resolution (in pixels)
// uniform vec4      iMouse;                // mouse pixel coords. xy: current (if MLB down), zw: click
// uniform samplerXX iChannel0..3;          // input channel. XX = 2D/Cube
// uniform vec4      iDate;                 // (year, month, day, time in seconds)
// uniform float     iSampleRate;           // sound sample rate (i.e., 44100)
void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
	float time = iTime;   
    vec2 p = (2.0 * fragCoord - iResolution.xy) / iResolution.y; // x:[-1.0; 1.0] left2right, y:[-1.0, 1.0] bottom2top
    
    // base camera parameters
    vec3 cam_target = vec3(0.0);
    vec3 cam_origin = vec3(1.0, 1.0, 2.0);

    // camera rotation with mouse
    vec2 mouse = (2.0 * iMouse.xy - iResolution.xy) / iResolution.xy;
    cam_origin.yz *= rotate(mouse.y * PI);
    cam_origin.xz *= rotate(mouse.x * PI);

    // rotate camera around scene
    cam_origin.xz *= rotate(time * 0.5);

    // camera matrix
    mat3 camera = camera(cam_origin, cam_target, 0.0);
    
    // final pixel color
    vec3 color = render(p, cam_origin, camera, time);

    fragColor = vec4(color, 1.0);
}
