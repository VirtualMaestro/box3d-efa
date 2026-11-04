// Flat C exports for the shared-frontend architecture.
//
// The TS frontend (src/frontend.ts) is the single source of the public API
// personality, shared byte-identical with native hosts (scriptc-game links
// Box3D as a static library and implements this same contract over FFI).
// Everything here is scalars: ids live in slot tables because a b3BodyId is
// 64 bits of index + world + generation, which does not survive a JS number.
//
// Reads go through a scratch row (b3f_body_read fills, b3f_getf indexes),
// so the contract stays pointer-free on every host.
//
// The embind layer (glue.cpp) is unchanged and still the published API; it
// shrinks away as the frontend reaches parity, one surface at a time.

#include <box3d/box3d.h>
#include <cstdint>
#include <emscripten/emscripten.h>
#define B3F EMSCRIPTEN_KEEPALIVE
extern "C"
{

// ---- handle tables ---------------------------------------------------------

#define B3F_MAX_BODIES 16384
#define B3F_MAX_SHAPES 16384

static b3BodyId b3f_bodies[B3F_MAX_BODIES];
static uint32_t b3f_body_freelist[B3F_MAX_BODIES];
static uint32_t b3f_body_top = 1; // slot 0 = null
static uint32_t b3f_body_nfree = 0;

static b3ShapeId b3f_shapes[B3F_MAX_SHAPES];
static uint32_t b3f_shape_freelist[B3F_MAX_SHAPES];
static uint32_t b3f_shape_top = 1;
static uint32_t b3f_shape_nfree = 0;

static uint32_t b3f_body_slot( b3BodyId id )
{
	uint32_t s;
	if ( b3f_body_nfree > 0 )
	{
		s = b3f_body_freelist[--b3f_body_nfree];
	}
	else
	{
		if ( b3f_body_top >= B3F_MAX_BODIES )
			return 0;
		s = b3f_body_top++;
	}
	b3f_bodies[s] = id;
	return s;
}

static uint32_t b3f_shape_slot( b3ShapeId id )
{
	uint32_t s;
	if ( b3f_shape_nfree > 0 )
	{
		s = b3f_shape_freelist[--b3f_shape_nfree];
	}
	else
	{
		if ( b3f_shape_top >= B3F_MAX_SHAPES )
			return 0;
		s = b3f_shape_top++;
	}
	b3f_shapes[s] = id;
	return s;
}

// ---- scratch reads ----------------------------------------------------------

static double b3f_scratch[12];

B3F double b3f_getf( int32_t i )
{
	if ( i < 0 || i >= 12 )
		return 0;
	return b3f_scratch[i];
}

// ---- world ------------------------------------------------------------------

B3F uint32_t b3f_world_create( double gx, double gy, double gz, uint32_t enableSleep, uint32_t workerCount )
{
	b3WorldDef def = b3DefaultWorldDef();
	def.gravity.x = (float)gx;
	def.gravity.y = (float)gy;
	def.gravity.z = (float)gz;
	def.enableSleep = enableSleep != 0;
#ifdef __EMSCRIPTEN_PTHREADS__
	// Box3D's in-tree scheduler (workerCount > 1, no callbacks) runs on
	// pthreads; the deluxe build has them, standard must stay serial.
	def.workerCount = workerCount > 0 ? workerCount : 1;
#else
	(void)workerCount;
#endif
	return b3StoreWorldId( b3CreateWorld( &def ) );
}

B3F void b3f_world_step( uint32_t w, double dt, int32_t substeps )
{
	b3World_Step( b3LoadWorldId( w ), (float)dt, substeps );
}

B3F void b3f_world_set_gravity( uint32_t w, double gx, double gy, double gz )
{
	b3Vec3 g = { (float)gx, (float)gy, (float)gz };
	b3World_SetGravity( b3LoadWorldId( w ), g );
}

B3F void b3f_world_destroy( uint32_t w )
{
	b3DestroyWorld( b3LoadWorldId( w ) );
	// Bodies and shapes died with the world. One live world is the current
	// frontend stance, so the tables reset with it.
	b3f_body_top = 1;
	b3f_body_nfree = 0;
	b3f_shape_top = 1;
	b3f_shape_nfree = 0;
}

// ---- bodies -----------------------------------------------------------------

B3F uint32_t b3f_body_create( uint32_t w, int32_t type, double px, double py, double pz, double qx, double qy, double qz,
							  double qw )
{
	b3BodyDef def = b3DefaultBodyDef();
	def.type = type == 2 ? b3_dynamicBody : type == 1 ? b3_kinematicBody : b3_staticBody;
	def.position.x = px;
	def.position.y = py;
	def.position.z = pz;
	def.rotation.v.x = (float)qx;
	def.rotation.v.y = (float)qy;
	def.rotation.v.z = (float)qz;
	def.rotation.s = (float)qw;
	return b3f_body_slot( b3CreateBody( b3LoadWorldId( w ), &def ) );
}

B3F void b3f_body_destroy( uint32_t b )
{
	if ( b == 0 || b >= b3f_body_top )
		return;
	b3DestroyBody( b3f_bodies[b] );
	b3f_body_freelist[b3f_body_nfree++] = b;
}

// Fills the scratch row: [px,py,pz, qx,qy,qz,qw, awake, vx,vy,vz].
B3F void b3f_body_read( uint32_t b )
{
	b3BodyId id = b3f_bodies[b];
	b3Pos p = b3Body_GetPosition( id );
	b3Quat q = b3Body_GetRotation( id );
	b3Vec3 v = b3Body_GetLinearVelocity( id );
	b3f_scratch[0] = p.x;
	b3f_scratch[1] = p.y;
	b3f_scratch[2] = p.z;
	b3f_scratch[3] = q.v.x;
	b3f_scratch[4] = q.v.y;
	b3f_scratch[5] = q.v.z;
	b3f_scratch[6] = q.s;
	b3f_scratch[7] = b3Body_IsAwake( id ) ? 1 : 0;
	b3f_scratch[8] = v.x;
	b3f_scratch[9] = v.y;
	b3f_scratch[10] = v.z;
}

// Move a body WITHOUT sweeping it there: restart/respawn teleports.
// Full pose: a respawned stack needs its rotations squared up too.
B3F void b3f_body_teleport( uint32_t b, double px, double py, double pz, double qx, double qy, double qz, double qw )
{
	b3BodyId id = b3f_bodies[b];
	b3Pos p;
	p.x = px;
	p.y = py;
	p.z = pz;
	b3Quat q;
	q.v.x = (float)qx;
	q.v.y = (float)qy;
	q.v.z = (float)qz;
	q.s = (float)qw;
	b3Body_SetTransform( id, p, q );
}

// Gravity scale (0 = flies) and an all-angular lock, for player hulls.
B3F void b3f_body_config( uint32_t b, double gravityScale, uint32_t lockAngular )
{
	b3BodyId id = b3f_bodies[b];
	b3Body_SetGravityScale( id, (float)gravityScale );
	b3MotionLocks locks = b3Body_GetMotionLocks( id );
	locks.angularX = lockAngular != 0;
	locks.angularY = lockAngular != 0;
	locks.angularZ = lockAngular != 0;
	b3Body_SetMotionLocks( id, locks );
}

B3F void b3f_body_set_velocity( uint32_t b, double vx, double vy, double vz )
{
	b3Vec3 v = { (float)vx, (float)vy, (float)vz };
	b3Body_SetLinearVelocity( b3f_bodies[b], v );
}

B3F void b3f_body_set_angular_velocity( uint32_t b, double wx, double wy, double wz )
{
	b3Vec3 v = { (float)wx, (float)wy, (float)wz };
	b3Body_SetAngularVelocity( b3f_bodies[b], v );
}

B3F void b3f_body_impulse( uint32_t b, double ix, double iy, double iz, uint32_t wake )
{
	b3Vec3 v = { (float)ix, (float)iy, (float)iz };
	b3Body_ApplyLinearImpulseToCenter( b3f_bodies[b], v, wake != 0 );
}

// ---- shapes -----------------------------------------------------------------

static b3ShapeDef b3f_shape_def( double density, double friction, double restitution )
{
	b3ShapeDef def = b3DefaultShapeDef();
	if ( density >= 0 )
		def.density = (float)density;
	if ( friction >= 0 )
		def.baseMaterial.friction = (float)friction;
	if ( restitution >= 0 )
		def.baseMaterial.restitution = (float)restitution;
	return def;
}

B3F uint32_t b3f_shape_box( uint32_t b, double hx, double hy, double hz, double density, double friction, double restitution )
{
	b3ShapeDef def = b3f_shape_def( density, friction, restitution );
	b3BoxHull hull = b3MakeBoxHull( (float)hx, (float)hy, (float)hz );
	return b3f_shape_slot( b3CreateHullShape( b3f_bodies[b], &def, &hull.base ) );
}

B3F uint32_t b3f_shape_sphere( uint32_t b, double radius, double density, double friction, double restitution )
{
	b3ShapeDef def = b3f_shape_def( density, friction, restitution );
	b3Sphere sphere = { { 0, 0, 0 }, (float)radius };
	return b3f_shape_slot( b3CreateSphereShape( b3f_bodies[b], &def, &sphere ) );
}

B3F void b3f_world_explode( uint32_t w, double px, double py, double pz, double radius, double falloff, double impulsePerArea )
{
	b3ExplosionDef def = b3DefaultExplosionDef();
	def.position.x = px;
	def.position.y = py;
	def.position.z = pz;
	def.radius = (float)radius;
	def.falloff = (float)falloff;
	def.impulsePerArea = (float)impulsePerArea;
	b3World_Explode( b3LoadWorldId( w ), &def );
}

} // extern "C"
