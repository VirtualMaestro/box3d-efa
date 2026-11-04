// embind glue for Box3D (https://github.com/erincatto/box3d).
// Box3D is Erin Catto's 3D physics engine. This file only adapts its C API
// to JavaScript; all engine credit belongs upstream.
//
// Conventions:
// - Vectors are plain JS objects {x, y, z}; quaternions are {x, y, z, w}.
// - Creation functions take a single options object applied over the Box3D
//   default def, so every field stays optional.
// - World/Body/Shape/Joint wrappers hold ids only. delete() frees the tiny
//   wrapper; destroy() removes the object from the simulation.
// - Bodies and shapes get an auto-assigned integer userData tag so event
//   arrays can be correlated back to application objects.

#include <box3d/box3d.h>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <emscripten/bind.h>
#include <emscripten/em_js.h>
#include <emscripten/val.h>
#include <string>
#include <unordered_map>
#include <vector>

using emscripten::EM_VAL;
using emscripten::val;

// ---------------------------------------------------------------------------
// val helpers
// ---------------------------------------------------------------------------

// Every call into JS costs a crossing, and val reads or writes a property by a C string key by decoding the string
// first (a crossing of its own) and freeing the handle after (another). Reading { x, y, z } the plain way took
// about 18 crossings. These helpers read a field, a vector or a quaternion in one crossing and build result objects
// in one; other property access goes through K, which makes each key's JS string once.

// Property keys as JS strings, made once and kept (string literals: one entry per literal).
static const val& K( const char* name )
{
	static std::unordered_map<const char*, val> keys;
	auto it = keys.find( name );
	if ( it == keys.end() )
	{
		it = keys.emplace( name, val::u8string( name ) ).first;
	}
	return it->second;
}

// The EM_JS bodies are JavaScript: clang-format leaves them alone.
// clang-format off
// o[key] as a number (+v), or fallback when o or o[key] is undefined or null.
EM_JS( double, b3g_number, ( EM_VAL o, EM_VAL key, double fallback ), {
	var v = Emval.toValue( o );
	if ( v === undefined || v === null ) return fallback;
	v = v[ Emval.toValue( key ) ];
	return v === undefined || v === null ? fallback : +v;
} );

// o[key] as an int32, or fallback. Saturating, as val::as<int> converted (a double cast to int): v | 0 wraps modulo
// 2^32, which turned a groupIndex of 2^31 or 0xffffffff negative (never collide instead of always).
EM_JS( int, b3g_int, ( EM_VAL o, EM_VAL key, int fallback ), {
	var v = Emval.toValue( o );
	if ( v === undefined || v === null ) return fallback;
	v = v[ Emval.toValue( key ) ];
	if ( v === undefined || v === null ) return fallback;
	v = +v;
	return v !== v ? 0 : v >= 2147483647 ? 2147483647 : v <= -2147483648 ? -2147483648 : v | 0;
} );

// o[key] as 0 or 1 (truthiness), or fallback when o or o[key] is undefined or null.
EM_JS( int, b3g_bool, ( EM_VAL o, EM_VAL key, int fallback ), {
	var v = Emval.toValue( o );
	if ( v === undefined || v === null ) return fallback;
	v = v[ Emval.toValue( key ) ];
	return v === undefined || v === null ? fallback : v ? 1 : 0;
} );

// o[key] (or o itself when key is 0) read as { x, y, z } (n 3) or { x, y, z, w } (n 4) into out; returns a bit per
// field present (x 1, y 2, z 4, w 8), 0 when the object is undefined or null. Fields not present are left as they are.
EM_JS( int, b3g_read, ( EM_VAL o, EM_VAL key, float* out, int n ), {
	var v = Emval.toValue( o );
	if ( key && v !== undefined && v !== null ) v = v[ Emval.toValue( key ) ];
	if ( v === undefined || v === null ) return 0;
	var i = out >> 2, m = 0, f;
	f = v.x; if ( f !== undefined && f !== null ) { HEAPF32[i] = f; m |= 1; }
	f = v.y; if ( f !== undefined && f !== null ) { HEAPF32[i + 1] = f; m |= 2; }
	f = v.z; if ( f !== undefined && f !== null ) { HEAPF32[i + 2] = f; m |= 4; }
	if ( n === 4 ) { f = v.w; if ( f !== undefined && f !== null ) { HEAPF32[i + 3] = f; m |= 8; } }
	return m;
} );

EM_JS( EM_VAL, b3g_vec3, ( float x, float y, float z ), { return Emval.toHandle( { x: x, y: y, z: z } ); } );

EM_JS( EM_VAL, b3g_quat, ( float x, float y, float z, float w ), { return Emval.toHandle( { x: x, y: y, z: z, w: w } ); } );

EM_JS( EM_VAL, b3g_transform, ( float px, float py, float pz, float qx, float qy, float qz, float qw ), {
	return Emval.toHandle( { position: { x: px, y: py, z: pz }, rotation: { x: qx, y: qy, z: qz, w: qw } } );
} );

// A method called on a destroyed body, shape or joint (or one of a destroyed world) throws (Guarded below).
EM_JS( void, b3g_stale, ( int kind ), {
	throw new Error( 'box3d: a ' + [ 'Body', 'Shape', 'Joint' ][ kind ] + ' handle used after its object or its world was destroyed' );
} );

// clang-format on

static EM_VAL keyHandle( const char* key )
{
	return key != nullptr ? K( key ).as_handle() : nullptr;
}

static bool hasKey( const val& o, const char* key )
{
	return b3g_bool( o.as_handle(), K( key ).as_handle(), -1 ) != -1;
}

static float getFloat( const val& o, const char* key, float fallback )
{
	return (float)b3g_number( o.as_handle(), K( key ).as_handle(), fallback );
}

static int getInt( const val& o, const char* key, int fallback )
{
	return b3g_int( o.as_handle(), K( key ).as_handle(), fallback );
}

static bool getBool( const val& o, const char* key, bool fallback )
{
	return b3g_bool( o.as_handle(), K( key ).as_handle(), fallback ? 1 : 0 ) != 0;
}

static uint64_t getU64( const val& o, const char* key, uint64_t fallback )
{
	// JS numbers are safe up to 2^53. Filter bits rarely need more. The fallback does not go through a double: the
	// default bits (2^64 - 1) round to 2^64, which does not convert back (the MVP flavour gave 0).
	double v = b3g_number( o.as_handle(), K( key ).as_handle(), NAN );
	return v == v ? (uint64_t)v : fallback;
}

static b3Vec3 readVec3( const val& o, const char* key, b3Vec3 fallback )
{
	b3Vec3 out = fallback;
	b3g_read( o.as_handle(), keyHandle( key ), &out.x, 3 );
	return out;
}

static b3Vec3 toVec3( const val& v, b3Vec3 fallback )
{
	return readVec3( v, nullptr, fallback );
}

static b3Vec3 getVec3( const val& o, const char* key, b3Vec3 fallback )
{
	return readVec3( o, key, fallback );
}

static val fromVec3( b3Vec3 v )
{
	return val::take_ownership( b3g_vec3( v.x, v.y, v.z ) );
}

static b3Quat readQuat( const val& o, const char* key, b3Quat fallback )
{
	float q[4] = { fallback.v.x, fallback.v.y, fallback.v.z, fallback.s };
	b3g_read( o.as_handle(), keyHandle( key ), q, 4 );
	return { { q[0], q[1], q[2] }, q[3] };
}

static b3Quat toQuat( const val& v, b3Quat fallback )
{
	return readQuat( v, nullptr, fallback );
}

static b3Quat getQuat( const val& o, const char* key, b3Quat fallback )
{
	return readQuat( o, key, fallback );
}

static val fromQuat( b3Quat q )
{
	return val::take_ownership( b3g_quat( q.v.x, q.v.y, q.v.z, q.s ) );
}

static b3Transform getTransform( const val& o, const char* key )
{
	b3Transform xf = b3Transform_identity;
	if ( hasKey( o, key ) )
	{
		val t = o[K( key )];
		xf.p = getVec3( t, "position", b3Vec3_zero );
		xf.q = getQuat( t, "rotation", b3Quat_identity );
	}
	return xf;
}

static val fromTransform( b3Transform xf )
{
	return val::take_ownership(
		b3g_transform( (float)xf.p.x, (float)xf.p.y, (float)xf.p.z, xf.q.v.x, xf.q.v.y, xf.q.v.z, xf.q.s ) );
}

static double tagOf( void* userData )
{
	return (double)(uintptr_t)userData;
}

// Auto-assigned userData tags. 0 is reserved for "untagged".
static uint32_t g_nextBodyTag = 1;
static uint32_t g_nextShapeTag = 1;

// ---------------------------------------------------------------------------
// Filters and materials
// ---------------------------------------------------------------------------

static b3Filter filterFromOpts( const val& o )
{
	b3Filter filter = b3DefaultFilter();
	if ( hasKey( o, "filter" ) )
	{
		val f = o[K( "filter" )];
		filter.categoryBits = getU64( f, "categoryBits", filter.categoryBits );
		filter.maskBits = getU64( f, "maskBits", filter.maskBits );
		filter.groupIndex = getInt( f, "groupIndex", filter.groupIndex );
	}
	return filter;
}

static b3QueryFilter queryFilterFromOpts( const val& o )
{
	b3QueryFilter filter = b3DefaultQueryFilter();
	if ( !o.isUndefined() && !o.isNull() )
	{
		filter.categoryBits = getU64( o, "categoryBits", filter.categoryBits );
		filter.maskBits = getU64( o, "maskBits", filter.maskBits );
	}
	return filter;
}

// ---------------------------------------------------------------------------
// Queries (ray and shape casts, overlap, closest point)
// ---------------------------------------------------------------------------

static_assert( sizeof( b3Pos ) == sizeof( b3Vec3 ), "the query glue assumes single precision positions" );

// A shape proxy from { points: [{x, y, z}, ...], radius }, relative to the query's origin. At most
// B3_MAX_SHAPE_CAST_POINTS points: a sphere is one point with a radius, a capsule two.
static b3ShapeProxy proxyFromOpts( const val& o, b3Vec3* points )
{
	val arr = o[K( "points" )];
	int n = arr[K( "length" )].as<int>();
	n = n < B3_MAX_SHAPE_CAST_POINTS ? n : B3_MAX_SHAPE_CAST_POINTS;
	for ( int i = 0; i < n; ++i )
	{
		points[i] = toVec3( arr[i], b3Vec3_zero );
	}
	b3ShapeProxy proxy = { points, n, getFloat( o, "radius", 0.0f ) };
	return proxy;
}

// World queries report sensor shapes; these glue queries leave them out unless the filter says includeSensors.
static bool includeSensorsOf( const val& filterOpts )
{
	return getBool( filterOpts, "includeSensors", false );
}

// The body a query hit: its index (Body.getIndex) and both user data tags.
static void setHitShape( val& o, b3ShapeId shapeId )
{
	b3BodyId body = b3Shape_GetBody( shapeId );
	o.set( K( "bodyIndex" ), body.index1 );
	o.set( K( "bodyUserData" ), tagOf( b3Body_GetUserData( body ) ) );
	o.set( K( "shapeUserData" ), tagOf( b3Shape_GetUserData( shapeId ) ) );
}

struct ClosestCastContext
{
	bool includeSensors;
	bool hit;
	b3ShapeId shapeId;
	b3Vec3 point;
	b3Vec3 normal;
	float fraction;
};

static float closestCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t, int, int, void* context )
{
	ClosestCastContext* c = (ClosestCastContext*)context;
	if ( !c->includeSensors && b3Shape_IsSensor( shapeId ) )
	{
		return -1.0f;
	}
	c->hit = true;
	c->shapeId = shapeId;
	c->point = point;
	c->normal = normal;
	c->fraction = fraction;
	return fraction;
}

static val closestCastResult( const ClosestCastContext& c )
{
	val o = val::object();
	o.set( K( "hit" ), c.hit );
	if ( c.hit )
	{
		o.set( K( "point" ), fromVec3( c.point ) );
		o.set( K( "normal" ), fromVec3( c.normal ) );
		o.set( K( "fraction" ), c.fraction );
		setHitShape( o, c.shapeId );
	}
	return o;
}

struct OverlapContext
{
	bool includeSensors;
	std::vector<b3ShapeId> shapes;
};

static bool overlapFcn( b3ShapeId shapeId, void* context )
{
	OverlapContext* c = (OverlapContext*)context;
	if ( c->includeSensors || !b3Shape_IsSensor( shapeId ) )
	{
		c->shapes.push_back( shapeId );
	}
	return true;
}

// The closest point on any shape within maxDistance of target, in one pass: the broadphase boxes within maxDistance,
// the closest point on each shape in the callback. The same shapes in the same tree order as an overlap with a
// sphere of radius maxDistance, then the closest point per shape (the two passes this replaced): the same answers,
// ties included. Convex shapes only (spheres, capsules, hulls): b3Shape_GetClosestPoint has no answer for a mesh, a
// height field or a compound (it gives the body's origin), and a box overlap reaches such shapes the sphere did not.
struct ClosestPointContext
{
	bool includeSensors;
	b3Vec3 target;
	float best;
	bool hit;
	b3Vec3 point;
	b3ShapeId shapeId;
};

static bool closestPointFcn( b3ShapeId shapeId, void* context )
{
	ClosestPointContext* c = (ClosestPointContext*)context;
	if ( !c->includeSensors && b3Shape_IsSensor( shapeId ) )
	{
		return true;
	}
	b3ShapeType type = b3Shape_GetType( shapeId );
	if ( type != b3_sphereShape && type != b3_capsuleShape && type != b3_hullShape )
	{
		return true;
	}
	b3Vec3 p = b3Shape_GetClosestPoint( shapeId, c->target );
	float d = b3Length( b3Sub( p, c->target ) );
	if ( d <= c->best && ( !c->hit || d < c->best ) )
	{
		c->hit = true;
		c->best = d;
		c->point = p;
		c->shapeId = shapeId;
	}
	return true;
}

static ClosestPointContext closestPointOf( b3WorldId world, b3Vec3 t, float maxDistance, b3QueryFilter filter,
										   bool includeSensors )
{
	ClosestPointContext c = { includeSensors, t, maxDistance, false, b3Vec3_zero, b3_nullShapeId };
	b3AABB box = { { t.x - maxDistance, t.y - maxDistance, t.z - maxDistance },
				   { t.x + maxDistance, t.y + maxDistance, t.z + maxDistance } };
	b3World_OverlapAABB( world, box, filter, closestPointFcn, &c );
	return c;
}

// ---------------------------------------------------------------------------
// Flat calls: records of 32-bit words in the module's memory, no JS objects
// ---------------------------------------------------------------------------

// The batched calls (World.queryBatch, readMoved, readEvents, writeBodies) read their input from one buffer and
// write their output to another, both in the module's memory: JS asks flatIn(words) for the input's byte offset,
// writes the records through HEAPU32 or HEAPF32, makes the call, then reads the output at flatOut(). The output is
// valid until the next flat call; memory can grow during any call, so JS takes HEAPU32 and HEAPF32 again after it.
// A word is a float or an unsigned integer, as each layout says. Body ids go as an index and a generation.
union FlatWord
{
	float f;
	uint32_t u;
};

static std::vector<FlatWord> g_flatIn, g_flatOut;

// Never address 0: an empty buffer still gets storage, so JS never reads or writes at the null page.
static int flatIn( int words )
{
	if ( words < 1 )
	{
		words = 1;
	}
	if ( words > (int)g_flatIn.size() )
	{
		g_flatIn.resize( words );
	}
	return (int)(uintptr_t)g_flatIn.data();
}

static int flatOut()
{
	if ( g_flatOut.capacity() == 0 )
	{
		g_flatOut.reserve( 16 );
	}
	return (int)(uintptr_t)g_flatOut.data();
}

static void putU( uint32_t u )
{
	FlatWord w;
	w.u = u;
	g_flatOut.push_back( w );
}

static void putF( float f )
{
	FlatWord w;
	w.f = f;
	g_flatOut.push_back( w );
}

static void putBody( b3BodyId body )
{
	putU( (uint32_t)body.index1 );
	putU( body.generation );
}

static void putShapeBody( b3ShapeId shape )
{
	if ( b3Shape_IsValid( shape ) )
	{
		putBody( b3Shape_GetBody( shape ) );
	}
	else
	{
		putU( UINT32_MAX );
		putU( UINT32_MAX );
	}
}

// ---------------------------------------------------------------------------
// The capsule mover (cast, collide, plane solver)
// ---------------------------------------------------------------------------

static b3Capsule capsuleFromOpts( const val& o )
{
	b3Capsule c;
	c.center1 = getVec3( o, "center1", b3Vec3_zero );
	c.center2 = getVec3( o, "center2", b3Vec3_zero );
	c.radius = getFloat( o, "radius", 0.0f );
	return c;
}

static bool moverFilterFcn( b3ShapeId shapeId, void* context )
{
	return *(bool*)context || !b3Shape_IsSensor( shapeId );
}

struct PlaneContext
{
	bool includeSensors;
	val out;
	int count;
};

// The planes a mover touches, kept in C for World.moverBatch: each plane with the shape it came from.
struct PlaneList
{
	bool includeSensors;
	std::vector<b3PlaneResult> planes;
	std::vector<b3ShapeId> shapes;
};

static bool planeListFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	PlaneList* c = (PlaneList*)context;
	if ( !c->includeSensors && b3Shape_IsSensor( shapeId ) )
	{
		return true;
	}
	for ( int i = 0; i < planeCount; ++i )
	{
		c->planes.push_back( planes[i] );
		c->shapes.push_back( shapeId );
	}
	return true;
}

static bool planeFcn( b3ShapeId shapeId, const b3PlaneResult* planes, int planeCount, void* context )
{
	PlaneContext* c = (PlaneContext*)context;
	if ( !c->includeSensors && b3Shape_IsSensor( shapeId ) )
	{
		return true;
	}
	for ( int i = 0; i < planeCount; ++i )
	{
		val o = val::object();
		o.set( K( "normal" ), fromVec3( planes[i].plane.normal ) );
		o.set( K( "offset" ), planes[i].plane.offset );
		o.set( K( "point" ), fromVec3( planes[i].point ) );
		setHitShape( o, shapeId );
		c->out.set( c->count++, o );
	}
	return true;
}

// Planes from { normal, offset, pushLimit (default FLT_MAX: rigid), clipVelocity (default true) }.
static std::vector<b3CollisionPlane> planesFromArray( const val& arr )
{
	int n = arr[K( "length" )].as<int>();
	std::vector<b3CollisionPlane> planes( n );
	for ( int i = 0; i < n; ++i )
	{
		val p = arr[i];
		planes[i].plane.normal = getVec3( p, "normal", b3Vec3_zero );
		planes[i].plane.offset = getFloat( p, "offset", 0.0f );
		planes[i].pushLimit = getFloat( p, "pushLimit", FLT_MAX );
		planes[i].push = getFloat( p, "push", 0.0f );
		planes[i].clipVelocity = getBool( p, "clipVelocity", true );
	}
	return planes;
}

// The mover's plane solver: the move closest to targetDelta that respects the planes (b3SolvePlanes). It also
// returns each plane's push, which clipVector reads (a plane that did not push does not clip).
static val solvePlanes( val targetDelta, val planesArr )
{
	std::vector<b3CollisionPlane> planes = planesFromArray( planesArr );
	b3PlaneSolverResult r = b3SolvePlanes( toVec3( targetDelta, b3Vec3_zero ), planes.data(), (int)planes.size() );
	val o = val::object();
	o.set( K( "delta" ), fromVec3( r.delta ) );
	o.set( K( "iterationCount" ), r.iterationCount );
	val push = val::array();
	for ( size_t i = 0; i < planes.size(); ++i )
	{
		push.set( i, planes[i].push );
	}
	o.set( K( "push" ), push );
	return o;
}

// A velocity clipped against the planes that pushed and have clipVelocity (b3ClipVector): pass each plane's push
// from solvePlanes.
static val clipVector( val vector, val planesArr )
{
	std::vector<b3CollisionPlane> planes = planesFromArray( planesArr );
	return fromVec3( b3ClipVector( toVec3( vector, b3Vec3_zero ), planes.data(), (int)planes.size() ) );
}

static b3ShapeDef shapeDefFromOpts( const val& o )
{
	b3ShapeDef def = b3DefaultShapeDef();
	def.userData = (void*)(uintptr_t)( g_nextShapeTag++ );
	if ( o.isUndefined() || o.isNull() )
	{
		return def;
	}
	def.density = getFloat( o, "density", def.density );
	def.baseMaterial.friction = getFloat( o, "friction", def.baseMaterial.friction );
	def.baseMaterial.restitution = getFloat( o, "restitution", def.baseMaterial.restitution );
	def.baseMaterial.rollingResistance = getFloat( o, "rollingResistance", def.baseMaterial.rollingResistance );
	def.baseMaterial.tangentVelocity = getVec3( o, "tangentVelocity", def.baseMaterial.tangentVelocity );
	def.baseMaterial.userMaterialId = getU64( o, "userMaterialId", def.baseMaterial.userMaterialId );
	def.isSensor = getBool( o, "isSensor", def.isSensor );
	def.enableSensorEvents = getBool( o, "enableSensorEvents", def.enableSensorEvents );
	def.enableContactEvents = getBool( o, "enableContactEvents", def.enableContactEvents );
	def.enableHitEvents = getBool( o, "enableHitEvents", def.enableHitEvents );
	def.invokeContactCreation = getBool( o, "invokeContactCreation", def.invokeContactCreation );
	def.updateBodyMass = getBool( o, "updateBodyMass", def.updateBodyMass );
	def.filter = filterFromOpts( o );
	if ( hasKey( o, "userData" ) )
	{
		def.userData = (void*)(uintptr_t)o[K( "userData" )].as<double>();
	}
	return def;
}

// ---------------------------------------------------------------------------
// Wrappers
// ---------------------------------------------------------------------------

// The generation of the world in each slot, as the glue made or restored it. A handle keeps the generation of the
// world it was made in, so a handle of a destroyed world is told apart from one of the world that took its slot (a
// body or shape id names its world by slot only).
static uint16_t g_worldGen[B3_MAX_WORLDS];

static uint16_t worldGenOf( uint16_t world0 )
{
	return world0 < B3_MAX_WORLDS ? g_worldGen[world0] : 0;
}

static bool worldLive( uint16_t world0, uint16_t generation )
{
	return b3World_IsValid( { (uint16_t)( world0 + 1 ), generation } );
}

// Every method of a Body, Shape or Joint handle except isValid, destroy, getIndex and getGeneration runs through
// Guarded<&Class::method>::call (EMSCRIPTEN_BINDINGS): the handle's check(), then the method. Box3D checks ids only in
// debug builds; a stale id read or wrote whatever held its slot (review of 2026-11-03).
template <auto M> struct Guarded;

template <typename C, typename R, typename... A, R ( C::*M )( A... )> struct Guarded<M>
{
	static R call( C& self, A... a )
	{
		self.check();
		return ( self.*M )( a... );
	}
};

template <typename C, typename R, typename... A, R ( C::*M )( A... ) const> struct Guarded<M>
{
	static R call( const C& self, A... a )
	{
		self.check();
		return ( self.*M )( a... );
	}
};

struct Shape
{
	b3ShapeId id;
	uint16_t worldGen = worldGenOf( id.world0 );

	void stamp()
	{
		worldGen = worldGenOf( id.world0 );
	}

	bool isValid() const
	{
		return worldLive( id.world0, worldGen ) && b3Shape_IsValid( id );
	}

	void check() const
	{
		if ( !isValid() )
		{
			b3g_stale( 1 );
		}
	}

	// A second destroy (or one after the body went) does nothing: Box3D checks ids only in debug builds, and a stale id
	// writes into whatever now holds its slot.
	void destroy( bool updateBodyMass )
	{
		if ( isValid() )
		{
			b3DestroyShape( id, updateBodyMass );
		}
		id = b3_nullShapeId;
	}

	std::string getType() const
	{
		switch ( b3Shape_GetType( id ) )
		{
			case b3_sphereShape:
				return "sphere";
			case b3_capsuleShape:
				return "capsule";
			case b3_hullShape:
				return "hull";
			case b3_meshShape:
				return "mesh";
			case b3_heightShape:
				return "heightField";
			case b3_compoundShape:
				return "compound";
			default:
				return "unknown";
		}
	}

	double getUserData() const
	{
		return tagOf( b3Shape_GetUserData( id ) );
	}

	void setUserData( double tag )
	{
		b3Shape_SetUserData( id, (void*)(uintptr_t)tag );
	}

	float getFriction() const
	{
		return b3Shape_GetFriction( id );
	}

	void setFriction( float friction )
	{
		b3Shape_SetFriction( id, friction );
	}

	float getRestitution() const
	{
		return b3Shape_GetRestitution( id );
	}

	void setRestitution( float restitution )
	{
		b3Shape_SetRestitution( id, restitution );
	}

	float getDensity() const
	{
		return b3Shape_GetDensity( id );
	}

	void setDensity( float density, bool updateBodyMass )
	{
		b3Shape_SetDensity( id, density, updateBodyMass );
	}

	bool isSensor() const
	{
		return b3Shape_IsSensor( id );
	}

	void enableSensorEvents( bool flag )
	{
		b3Shape_EnableSensorEvents( id, flag );
	}

	void enableContactEvents( bool flag )
	{
		b3Shape_EnableContactEvents( id, flag );
	}

	// The shape's touching contacts (b3Shape_GetContactData): an array of { bodyIndexA, bodyIndexB, shapeUserDataA,
	// shapeUserDataB, manifolds: [{ normal, points: [{ anchorA, anchorB, separation, normalImpulse,
	// totalNormalImpulse, normalVelocity }] }] }. Anchors are relative to each body's center of mass.
	val getContactData() const
	{
		int capacity = b3Shape_GetContactCapacity( id );
		std::vector<b3ContactData> data( capacity > 0 ? capacity : 0 );
		int n = capacity > 0 ? b3Shape_GetContactData( id, data.data(), capacity ) : 0;
		val out = val::array();
		for ( int i = 0; i < n; ++i )
		{
			const b3ContactData& c = data[i];
			val o = val::object();
			o.set( K( "bodyIndexA" ), b3Shape_GetBody( c.shapeIdA ).index1 );
			o.set( K( "bodyIndexB" ), b3Shape_GetBody( c.shapeIdB ).index1 );
			o.set( K( "shapeUserDataA" ), tagOf( b3Shape_GetUserData( c.shapeIdA ) ) );
			o.set( K( "shapeUserDataB" ), tagOf( b3Shape_GetUserData( c.shapeIdB ) ) );
			val manifolds = val::array();
			for ( int m = 0; m < c.manifoldCount; ++m )
			{
				const b3Manifold& mf = c.manifolds[m];
				val mo = val::object();
				mo.set( K( "normal" ), fromVec3( mf.normal ) );
				val points = val::array();
				for ( int k = 0; k < mf.pointCount; ++k )
				{
					const b3ManifoldPoint& mp = mf.points[k];
					val po = val::object();
					po.set( K( "anchorA" ), fromVec3( mp.anchorA ) );
					po.set( K( "anchorB" ), fromVec3( mp.anchorB ) );
					po.set( K( "separation" ), mp.separation );
					po.set( K( "normalImpulse" ), mp.normalImpulse );
					po.set( K( "totalNormalImpulse" ), mp.totalNormalImpulse );
					po.set( K( "normalVelocity" ), mp.normalVelocity );
					points.set( k, po );
				}
				mo.set( K( "points" ), points );
				manifolds.set( m, mo );
			}
			o.set( K( "manifolds" ), manifolds );
			out.set( i, o );
		}
		return out;
	}

	void enableHitEvents( bool flag )
	{
		b3Shape_EnableHitEvents( id, flag );
	}

	val getFilter() const
	{
		b3Filter f = b3Shape_GetFilter( id );
		val o = val::object();
		o.set( K( "categoryBits" ), (double)f.categoryBits );
		o.set( K( "maskBits" ), (double)f.maskBits );
		o.set( K( "groupIndex" ), f.groupIndex );
		return o;
	}

	void setFilter( val opts )
	{
		b3Filter f = b3Shape_GetFilter( id );
		f.categoryBits = getU64( opts, "categoryBits", f.categoryBits );
		f.maskBits = getU64( opts, "maskBits", f.maskBits );
		f.groupIndex = getInt( opts, "groupIndex", f.groupIndex );
		b3Shape_SetFilter( id, f, true );
	}

	val getAABB() const
	{
		b3AABB aabb = b3Shape_GetAABB( id );
		val o = val::object();
		o.set( K( "lowerBound" ), fromVec3( aabb.lowerBound ) );
		o.set( K( "upperBound" ), fromVec3( aabb.upperBound ) );
		return o;
	}

	// The point on this shape closest to target, in world space (target itself when it is inside a solid shape).
	val getClosestPoint( val target ) const
	{
		return fromVec3( b3Shape_GetClosestPoint( id, toVec3( target, b3Vec3_zero ) ) );
	}

	val rayCast( val origin, val translation ) const
	{
		b3WorldCastOutput out = b3Shape_RayCast( id, toVec3( origin, b3Vec3_zero ), toVec3( translation, b3Vec3_zero ) );
		val o = val::object();
		o.set( K( "hit" ), out.hit );
		if ( out.hit )
		{
			o.set( K( "point" ), fromVec3( out.point ) );
			o.set( K( "normal" ), fromVec3( out.normal ) );
			o.set( K( "fraction" ), out.fraction );
		}
		return o;
	}
};

struct Joint
{
	b3JointId id;
	uint16_t worldGen = worldGenOf( id.world0 );

	void stamp()
	{
		worldGen = worldGenOf( id.world0 );
	}

	bool isValid() const
	{
		return worldLive( id.world0, worldGen ) && b3Joint_IsValid( id );
	}

	void check() const
	{
		if ( !isValid() )
		{
			b3g_stale( 2 );
		}
	}

	// A second destroy (or one after a body took the joint along) does nothing, as for shapes.
	void destroy( bool wakeAttached )
	{
		if ( isValid() )
		{
			b3DestroyJoint( id, wakeAttached );
		}
		id = b3_nullJointId;
	}

	std::string getType() const
	{
		switch ( b3Joint_GetType( id ) )
		{
			case b3_distanceJoint:
				return "distance";
			case b3_filterJoint:
				return "filter";
			case b3_motorJoint:
				return "motor";
			case b3_parallelJoint:
				return "parallel";
			case b3_prismaticJoint:
				return "prismatic";
			case b3_revoluteJoint:
				return "revolute";
			case b3_sphericalJoint:
				return "spherical";
			case b3_weldJoint:
				return "weld";
			case b3_wheelJoint:
				return "wheel";
			default:
				return "unknown";
		}
	}

	void wakeBodies()
	{
		b3Joint_WakeBodies( id );
	}

	bool getCollideConnected() const
	{
		return b3Joint_GetCollideConnected( id );
	}

	void setCollideConnected( bool flag )
	{
		b3Joint_SetCollideConnected( id, flag );
	}

	val getLocalFrameA() const
	{
		return fromTransform( b3Joint_GetLocalFrameA( id ) );
	}

	val getLocalFrameB() const
	{
		return fromTransform( b3Joint_GetLocalFrameB( id ) );
	}

	val getConstraintForce() const
	{
		return fromVec3( b3Joint_GetConstraintForce( id ) );
	}

	val getConstraintTorque() const
	{
		return fromVec3( b3Joint_GetConstraintTorque( id ) );
	}

	// A joint event (World.getJointEvents) is raised in a step where the constraint force or torque exceeds its
	// threshold. The thresholds default to FLT_MAX: no events.
	void setForceThreshold( float threshold )
	{
		b3Joint_SetForceThreshold( id, threshold );
	}

	void setTorqueThreshold( float threshold )
	{
		b3Joint_SetTorqueThreshold( id, threshold );
	}

	// The joint's index, as a joint event names it.
	int getIndex() const
	{
		return id.index1;
	}
};

struct DistanceJoint : Joint
{
	void setLength( float length )
	{
		b3DistanceJoint_SetLength( id, length );
	}
	float getLength() const
	{
		return b3DistanceJoint_GetLength( id );
	}
	float getCurrentLength() const
	{
		return b3DistanceJoint_GetCurrentLength( id );
	}
	void enableSpring( bool flag )
	{
		b3DistanceJoint_EnableSpring( id, flag );
	}
	void setSpringHertz( float hertz )
	{
		b3DistanceJoint_SetSpringHertz( id, hertz );
	}
	void setSpringDampingRatio( float ratio )
	{
		b3DistanceJoint_SetSpringDampingRatio( id, ratio );
	}
	void enableLimit( bool flag )
	{
		b3DistanceJoint_EnableLimit( id, flag );
	}
	void setLengthRange( float minLength, float maxLength )
	{
		b3DistanceJoint_SetLengthRange( id, minLength, maxLength );
	}
	void enableMotor( bool flag )
	{
		b3DistanceJoint_EnableMotor( id, flag );
	}
	void setMotorSpeed( float speed )
	{
		b3DistanceJoint_SetMotorSpeed( id, speed );
	}
	void setMaxMotorForce( float force )
	{
		b3DistanceJoint_SetMaxMotorForce( id, force );
	}
};

struct RevoluteJoint : Joint
{
	float getAngle() const
	{
		return b3RevoluteJoint_GetAngle( id );
	}
	void enableSpring( bool flag )
	{
		b3RevoluteJoint_EnableSpring( id, flag );
	}
	void setSpringHertz( float hertz )
	{
		b3RevoluteJoint_SetSpringHertz( id, hertz );
	}
	void setSpringDampingRatio( float ratio )
	{
		b3RevoluteJoint_SetSpringDampingRatio( id, ratio );
	}
	void setTargetAngle( float radians )
	{
		b3RevoluteJoint_SetTargetAngle( id, radians );
	}
	void enableLimit( bool flag )
	{
		b3RevoluteJoint_EnableLimit( id, flag );
	}
	void setLimits( float lower, float upper )
	{
		b3RevoluteJoint_SetLimits( id, lower, upper );
	}
	void enableMotor( bool flag )
	{
		b3RevoluteJoint_EnableMotor( id, flag );
	}
	void setMotorSpeed( float speed )
	{
		b3RevoluteJoint_SetMotorSpeed( id, speed );
	}
	void setMaxMotorTorque( float torque )
	{
		b3RevoluteJoint_SetMaxMotorTorque( id, torque );
	}
	float getMotorTorque() const
	{
		return b3RevoluteJoint_GetMotorTorque( id );
	}
};

struct SphericalJoint : Joint
{
	void enableConeLimit( bool flag )
	{
		b3SphericalJoint_EnableConeLimit( id, flag );
	}
	void setConeLimit( float radians )
	{
		b3SphericalJoint_SetConeLimit( id, radians );
	}
	float getConeAngle() const
	{
		return b3SphericalJoint_GetConeAngle( id );
	}
	void enableTwistLimit( bool flag )
	{
		b3SphericalJoint_EnableTwistLimit( id, flag );
	}
	void setTwistLimits( float lower, float upper )
	{
		b3SphericalJoint_SetTwistLimits( id, lower, upper );
	}
	float getTwistAngle() const
	{
		return b3SphericalJoint_GetTwistAngle( id );
	}
	void enableSpring( bool flag )
	{
		b3SphericalJoint_EnableSpring( id, flag );
	}
	void setSpringHertz( float hertz )
	{
		b3SphericalJoint_SetSpringHertz( id, hertz );
	}
	void setSpringDampingRatio( float ratio )
	{
		b3SphericalJoint_SetSpringDampingRatio( id, ratio );
	}
	void setTargetRotation( val q )
	{
		b3SphericalJoint_SetTargetRotation( id, toQuat( q, b3Quat_identity ) );
	}
	void enableMotor( bool flag )
	{
		b3SphericalJoint_EnableMotor( id, flag );
	}
	void setMotorVelocity( val v )
	{
		b3SphericalJoint_SetMotorVelocity( id, toVec3( v, b3Vec3_zero ) );
	}
	void setMaxMotorTorque( float torque )
	{
		b3SphericalJoint_SetMaxMotorTorque( id, torque );
	}
};

struct PrismaticJoint : Joint
{
	float getTranslation() const
	{
		return b3PrismaticJoint_GetTranslation( id );
	}
	float getSpeed() const
	{
		return b3PrismaticJoint_GetSpeed( id );
	}
	void enableSpring( bool flag )
	{
		b3PrismaticJoint_EnableSpring( id, flag );
	}
	void setSpringHertz( float hertz )
	{
		b3PrismaticJoint_SetSpringHertz( id, hertz );
	}
	void setSpringDampingRatio( float ratio )
	{
		b3PrismaticJoint_SetSpringDampingRatio( id, ratio );
	}
	void setTargetTranslation( float translation )
	{
		b3PrismaticJoint_SetTargetTranslation( id, translation );
	}
	void enableLimit( bool flag )
	{
		b3PrismaticJoint_EnableLimit( id, flag );
	}
	void setLimits( float lower, float upper )
	{
		b3PrismaticJoint_SetLimits( id, lower, upper );
	}
	void enableMotor( bool flag )
	{
		b3PrismaticJoint_EnableMotor( id, flag );
	}
	void setMotorSpeed( float speed )
	{
		b3PrismaticJoint_SetMotorSpeed( id, speed );
	}
	void setMaxMotorForce( float force )
	{
		b3PrismaticJoint_SetMaxMotorForce( id, force );
	}
};

struct WeldJoint : Joint
{
	void setLinearHertz( float hertz )
	{
		b3WeldJoint_SetLinearHertz( id, hertz );
	}
	void setLinearDampingRatio( float ratio )
	{
		b3WeldJoint_SetLinearDampingRatio( id, ratio );
	}
	void setAngularHertz( float hertz )
	{
		b3WeldJoint_SetAngularHertz( id, hertz );
	}
	void setAngularDampingRatio( float ratio )
	{
		b3WeldJoint_SetAngularDampingRatio( id, ratio );
	}
};

struct MotorJoint : Joint
{
	void setLinearVelocity( val v )
	{
		b3MotorJoint_SetLinearVelocity( id, toVec3( v, b3Vec3_zero ) );
	}
	void setAngularVelocity( val v )
	{
		b3MotorJoint_SetAngularVelocity( id, toVec3( v, b3Vec3_zero ) );
	}
	void setMaxVelocityForce( float force )
	{
		b3MotorJoint_SetMaxVelocityForce( id, force );
	}
	void setMaxVelocityTorque( float torque )
	{
		b3MotorJoint_SetMaxVelocityTorque( id, torque );
	}
	void setLinearHertz( float hertz )
	{
		b3MotorJoint_SetLinearHertz( id, hertz );
	}
	void setLinearDampingRatio( float ratio )
	{
		b3MotorJoint_SetLinearDampingRatio( id, ratio );
	}
	void setAngularHertz( float hertz )
	{
		b3MotorJoint_SetAngularHertz( id, hertz );
	}
	void setAngularDampingRatio( float ratio )
	{
		b3MotorJoint_SetAngularDampingRatio( id, ratio );
	}
	void setMaxSpringForce( float force )
	{
		b3MotorJoint_SetMaxSpringForce( id, force );
	}
	void setMaxSpringTorque( float torque )
	{
		b3MotorJoint_SetMaxSpringTorque( id, torque );
	}
};

struct WheelJoint : Joint
{
	void enableSuspension( bool flag )
	{
		b3WheelJoint_EnableSuspension( id, flag );
	}
	void setSuspensionHertz( float hertz )
	{
		b3WheelJoint_SetSuspensionHertz( id, hertz );
	}
	void setSuspensionDampingRatio( float ratio )
	{
		b3WheelJoint_SetSuspensionDampingRatio( id, ratio );
	}
	void enableSuspensionLimit( bool flag )
	{
		b3WheelJoint_EnableSuspensionLimit( id, flag );
	}
	void setSuspensionLimits( float lower, float upper )
	{
		b3WheelJoint_SetSuspensionLimits( id, lower, upper );
	}
	void enableSpinMotor( bool flag )
	{
		b3WheelJoint_EnableSpinMotor( id, flag );
	}
	void setSpinMotorSpeed( float speed )
	{
		b3WheelJoint_SetSpinMotorSpeed( id, speed );
	}
	void setMaxSpinTorque( float torque )
	{
		b3WheelJoint_SetMaxSpinTorque( id, torque );
	}
	float getSpinSpeed() const
	{
		return b3WheelJoint_GetSpinSpeed( id );
	}
	void enableSteering( bool flag )
	{
		b3WheelJoint_EnableSteering( id, flag );
	}
	void setSteeringHertz( float hertz )
	{
		b3WheelJoint_SetSteeringHertz( id, hertz );
	}
	void setSteeringDampingRatio( float ratio )
	{
		b3WheelJoint_SetSteeringDampingRatio( id, ratio );
	}
	void setMaxSteeringTorque( float torque )
	{
		b3WheelJoint_SetMaxSteeringTorque( id, torque );
	}
	void enableSteeringLimit( bool flag )
	{
		b3WheelJoint_EnableSteeringLimit( id, flag );
	}
	void setSteeringLimits( float lower, float upper )
	{
		b3WheelJoint_SetSteeringLimits( id, lower, upper );
	}
	void setTargetSteeringAngle( float radians )
	{
		b3WheelJoint_SetTargetSteeringAngle( id, radians );
	}
	float getSteeringAngle() const
	{
		return b3WheelJoint_GetSteeringAngle( id );
	}
};

struct ParallelJoint : Joint
{
	void setSpringHertz( float hertz )
	{
		b3ParallelJoint_SetSpringHertz( id, hertz );
	}
	void setSpringDampingRatio( float ratio )
	{
		b3ParallelJoint_SetSpringDampingRatio( id, ratio );
	}
	void setMaxTorque( float torque )
	{
		b3ParallelJoint_SetMaxTorque( id, torque );
	}
};

struct FilterJoint : Joint
{
};

struct Body
{
	b3BodyId id;
	uint16_t worldGen = worldGenOf( id.world0 );

	void stamp()
	{
		worldGen = worldGenOf( id.world0 );
	}

	bool isValid() const
	{
		return worldLive( id.world0, worldGen ) && b3Body_IsValid( id );
	}

	void check() const
	{
		if ( !isValid() )
		{
			b3g_stale( 0 );
		}
	}

	// A second destroy does nothing, as for shapes: before, it destroyed whatever held slot 0 of world 0.
	void destroy()
	{
		if ( isValid() )
		{
			b3DestroyBody( id );
		}
		id = b3_nullBodyId;
	}

	std::string getType() const
	{
		switch ( b3Body_GetType( id ) )
		{
			case b3_staticBody:
				return "static";
			case b3_kinematicBody:
				return "kinematic";
			case b3_dynamicBody:
				return "dynamic";
			default:
				return "unknown";
		}
	}

	void setType( std::string type )
	{
		b3BodyType t = b3_staticBody;
		if ( type == "dynamic" )
		{
			t = b3_dynamicBody;
		}
		else if ( type == "kinematic" )
		{
			t = b3_kinematicBody;
		}
		b3Body_SetType( id, t );
	}

	std::string getName() const
	{
		const char* name = b3Body_GetName( id );
		return name != nullptr ? name : "";
	}

	void setName( std::string name )
	{
		b3Body_SetName( id, name.c_str() );
	}

	double getUserData() const
	{
		return tagOf( b3Body_GetUserData( id ) );
	}

	void setUserData( double tag )
	{
		b3Body_SetUserData( id, (void*)(uintptr_t)tag );
	}

	// The body's id as numbers. Query hits name their body by this index (bodyIndex); the generation tells apart
	// bodies that reuse an index.
	int getIndex() const
	{
		return id.index1;
	}

	int getGeneration() const
	{
		return id.generation;
	}

	val getPosition() const
	{
		return fromVec3( b3Body_GetPosition( id ) );
	}

	val getRotation() const
	{
		return fromQuat( b3Body_GetRotation( id ) );
	}

	val getTransform() const
	{
		return fromTransform( b3Body_GetTransform( id ) );
	}

	void setTransform( val position, val rotation )
	{
		b3Body_SetTransform( id, toVec3( position, b3Body_GetPosition( id ) ), toQuat( rotation, b3Body_GetRotation( id ) ) );
	}

	void setTargetTransform( val target, float timeStep, bool wake )
	{
		b3WorldTransform xf;
		xf.p = getVec3( target, "position", b3Body_GetPosition( id ) );
		xf.q = getQuat( target, "rotation", b3Body_GetRotation( id ) );
		b3Body_SetTargetTransform( id, xf, timeStep, wake );
	}

	val getLinearVelocity() const
	{
		return fromVec3( b3Body_GetLinearVelocity( id ) );
	}

	void setLinearVelocity( val v )
	{
		b3Body_SetLinearVelocity( id, toVec3( v, b3Vec3_zero ) );
	}

	val getAngularVelocity() const
	{
		return fromVec3( b3Body_GetAngularVelocity( id ) );
	}

	void setAngularVelocity( val v )
	{
		b3Body_SetAngularVelocity( id, toVec3( v, b3Vec3_zero ) );
	}

	void applyForce( val force, val point, bool wake )
	{
		b3Body_ApplyForce( id, toVec3( force, b3Vec3_zero ), toVec3( point, b3Vec3_zero ), wake );
	}

	void applyForceToCenter( val force, bool wake )
	{
		b3Body_ApplyForceToCenter( id, toVec3( force, b3Vec3_zero ), wake );
	}

	void applyTorque( val torque, bool wake )
	{
		b3Body_ApplyTorque( id, toVec3( torque, b3Vec3_zero ), wake );
	}

	void applyLinearImpulse( val impulse, val point, bool wake )
	{
		b3Body_ApplyLinearImpulse( id, toVec3( impulse, b3Vec3_zero ), toVec3( point, b3Vec3_zero ), wake );
	}

	void applyLinearImpulseToCenter( val impulse, bool wake )
	{
		b3Body_ApplyLinearImpulseToCenter( id, toVec3( impulse, b3Vec3_zero ), wake );
	}

	void applyAngularImpulse( val impulse, bool wake )
	{
		b3Body_ApplyAngularImpulse( id, toVec3( impulse, b3Vec3_zero ), wake );
	}

	float getMass() const
	{
		return b3Body_GetMass( id );
	}

	void applyMassFromShapes()
	{
		b3Body_ApplyMassFromShapes( id );
	}

	val getLocalCenterOfMass() const
	{
		return fromVec3( b3Body_GetLocalCenterOfMass( id ) );
	}

	val getWorldCenterOfMass() const
	{
		return fromVec3( b3Body_GetWorldCenterOfMass( id ) );
	}

	val getLocalPoint( val worldPoint ) const
	{
		return fromVec3( b3Body_GetLocalPoint( id, toVec3( worldPoint, b3Vec3_zero ) ) );
	}

	val getWorldPoint( val localPoint ) const
	{
		return fromVec3( b3Body_GetWorldPoint( id, toVec3( localPoint, b3Vec3_zero ) ) );
	}

	float getLinearDamping() const
	{
		return b3Body_GetLinearDamping( id );
	}

	void setLinearDamping( float damping )
	{
		b3Body_SetLinearDamping( id, damping );
	}

	float getAngularDamping() const
	{
		return b3Body_GetAngularDamping( id );
	}

	void setAngularDamping( float damping )
	{
		b3Body_SetAngularDamping( id, damping );
	}

	float getGravityScale() const
	{
		return b3Body_GetGravityScale( id );
	}

	void setGravityScale( float scale )
	{
		b3Body_SetGravityScale( id, scale );
	}

	bool isAwake() const
	{
		return b3Body_IsAwake( id );
	}

	void setAwake( bool awake )
	{
		b3Body_SetAwake( id, awake );
	}

	void enableSleep( bool flag )
	{
		b3Body_EnableSleep( id, flag );
	}

	bool isEnabled() const
	{
		return b3Body_IsEnabled( id );
	}

	void setEnabled( bool flag )
	{
		if ( flag )
		{
			b3Body_Enable( id );
		}
		else
		{
			b3Body_Disable( id );
		}
	}

	bool isBullet() const
	{
		return b3Body_IsBullet( id );
	}

	void setBullet( bool flag )
	{
		b3Body_SetBullet( id, flag );
	}

	void setMotionLocks( val locks )
	{
		b3MotionLocks ml = b3Body_GetMotionLocks( id );
		ml.linearX = getBool( locks, "linearX", ml.linearX );
		ml.linearY = getBool( locks, "linearY", ml.linearY );
		ml.linearZ = getBool( locks, "linearZ", ml.linearZ );
		ml.angularX = getBool( locks, "angularX", ml.angularX );
		ml.angularY = getBool( locks, "angularY", ml.angularY );
		ml.angularZ = getBool( locks, "angularZ", ml.angularZ );
		b3Body_SetMotionLocks( id, ml );
	}

	val getMotionLocks() const
	{
		b3MotionLocks ml = b3Body_GetMotionLocks( id );
		val o = val::object();
		o.set( K( "linearX" ), ml.linearX );
		o.set( K( "linearY" ), ml.linearY );
		o.set( K( "linearZ" ), ml.linearZ );
		o.set( K( "angularX" ), ml.angularX );
		o.set( K( "angularY" ), ml.angularY );
		o.set( K( "angularZ" ), ml.angularZ );
		return o;
	}

	int getShapeCount() const
	{
		return b3Body_GetShapeCount( id );
	}

	val computeAABB() const
	{
		b3AABB aabb = b3Body_ComputeAABB( id );
		val o = val::object();
		o.set( K( "lowerBound" ), fromVec3( aabb.lowerBound ) );
		o.set( K( "upperBound" ), fromVec3( aabb.upperBound ) );
		return o;
	}

	// Shape creation. One options object carries both geometry and material.

	Shape createSphere( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		b3Sphere sphere;
		sphere.center = getVec3( opts, "center", b3Vec3_zero );
		sphere.radius = getFloat( opts, "radius", 0.5f );
		Shape shape = { b3CreateSphereShape( id, &def, &sphere ) };
		return shape;
	}

	Shape createCapsule( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		b3Capsule capsule;
		capsule.radius = getFloat( opts, "radius", 0.5f );
		if ( hasKey( opts, "height" ) )
		{
			// height is the segment length between the two hemisphere centers,
			// aligned with the local y axis.
			float half = 0.5f * opts[K( "height" )].as<float>();
			capsule.center1 = { 0.0f, -half, 0.0f };
			capsule.center2 = { 0.0f, half, 0.0f };
		}
		else
		{
			b3Vec3 c1 = { 0.0f, -0.5f, 0.0f };
			b3Vec3 c2 = { 0.0f, 0.5f, 0.0f };
			capsule.center1 = getVec3( opts, "center1", c1 );
			capsule.center2 = getVec3( opts, "center2", c2 );
		}
		Shape shape = { b3CreateCapsuleShape( id, &def, &capsule ) };
		return shape;
	}

	Shape createBox( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		b3Vec3 he = { 0.5f, 0.5f, 0.5f };
		if ( hasKey( opts, "halfExtents" ) )
		{
			he = toVec3( opts[K( "halfExtents" )], he );
		}
		else if ( hasKey( opts, "hx" ) || hasKey( opts, "hy" ) || hasKey( opts, "hz" ) )
		{
			he.x = getFloat( opts, "hx", 0.5f );
			he.y = getFloat( opts, "hy", 0.5f );
			he.z = getFloat( opts, "hz", 0.5f );
		}

		b3BoxHull box;
		if ( hasKey( opts, "rotation" ) || hasKey( opts, "offset" ) )
		{
			b3Transform xf;
			xf.p = getVec3( opts, "offset", b3Vec3_zero );
			xf.q = getQuat( opts, "rotation", b3Quat_identity );
			box = b3MakeTransformedBoxHull( he.x, he.y, he.z, xf );
		}
		else
		{
			box = b3MakeBoxHull( he.x, he.y, he.z );
		}
		Shape shape = { b3CreateHullShape( id, &def, &box.base ) };
		return shape;
	}

	// A triangle mesh from flat arrays (vertices: x y z per vertex; indices: three per triangle), for static terrain: Box3D
	// meshes collide with static bodies only. Box3D keeps a pointer to the mesh data, so it is never freed.
	Shape createMesh( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		val vs = opts[K( "vertices" )], is = opts[K( "indices" )];
		int nv = vs[K( "length" )].as<int>() / 3, ni = is[K( "length" )].as<int>();
		std::vector<b3Vec3> vertices( nv );
		for ( int i = 0; i < nv; ++i )
			vertices[i] = { vs[3 * i].as<float>(), vs[3 * i + 1].as<float>(), vs[3 * i + 2].as<float>() };
		std::vector<int32_t> indices( ni );
		for ( int i = 0; i < ni; ++i )
			indices[i] = is[i].as<int32_t>();
		std::vector<uint8_t> materials( ni / 3, 0 );
		b3MeshDef md = {};
		md.vertices = vertices.data();
		md.indices = indices.data();
		md.materialIndices = materials.data();
		md.vertexCount = nv;
		md.triangleCount = ni / 3;
		md.useMedianSplit = true;
		md.identifyEdges = true;
		b3MeshData* mesh = b3CreateMesh( &md, nullptr, 0 );
		if ( mesh == nullptr )
		{
			Shape shape = { b3_nullShapeId };
			return shape;
		}
		Shape shape = { b3CreateMeshShape( id, &def, mesh, { 1.0f, 1.0f, 1.0f } ) };
		return shape;
	}

	Shape createHull( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		std::vector<b3Vec3> points;
		val arr = opts[K( "points" )];
		int n = arr[K( "length" )].as<int>();
		points.reserve( n );
		for ( int i = 0; i < n; ++i )
		{
			points.push_back( toVec3( arr[i], b3Vec3_zero ) );
		}
		int maxVertices = getInt( opts, "maxVertices", 32 );
		b3HullData* hull = b3CreateHull( points.data(), (int)points.size(), maxVertices );
		if ( hull == nullptr )
		{
			Shape shape = { b3_nullShapeId };
			return shape;
		}
		// The world interns hull geometry in its hull database, so the
		// temporary may be freed right after shape creation.
		Shape shape = { b3CreateHullShape( id, &def, hull ) };
		b3DestroyHull( hull );
		return shape;
	}

	// A height field from a flat array of heights, row by row: heights[z * countX + x]. The grid starts at the body's
	// origin and spans (countX - 1) * scale.x along x and (countZ - 1) * scale.z along z. Heights are quantized to 16
	// bits between globalMinimumHeight and globalMaximumHeight (default: the heights' own range). The shape keeps a
	// pointer to the data, so the data lives as long as the module.
	Shape createHeightField( val opts )
	{
		b3ShapeDef def = shapeDefFromOpts( opts );
		std::vector<float> heights = emscripten::convertJSArrayToNumberVector<float>( opts[K( "heights" )] );
		b3HeightFieldDef hd = {};
		hd.countX = getInt( opts, "countX", 0 );
		hd.countZ = getInt( opts, "countZ", 0 );
		if ( hd.countX < 2 || hd.countZ < 2 || (int)heights.size() != hd.countX * hd.countZ )
		{
			Shape shape = { b3_nullShapeId };
			return shape;
		}
		float lo = heights[0], hi = heights[0];
		for ( float h : heights )
		{
			lo = h < lo ? h : lo;
			hi = h > hi ? h : hi;
		}
		std::vector<uint8_t> materials;
		if ( hasKey( opts, "materialIndices" ) )
		{
			materials = emscripten::convertJSArrayToNumberVector<uint8_t>( opts[K( "materialIndices" )] );
		}
		hd.heights = heights.data();
		hd.materialIndices = (int)materials.size() == ( hd.countX - 1 ) * ( hd.countZ - 1 ) ? materials.data() : nullptr;
		hd.scale = getVec3( opts, "scale", { 1.0f, 1.0f, 1.0f } );
		hd.globalMinimumHeight = getFloat( opts, "globalMinimumHeight", lo );
		hd.globalMaximumHeight = getFloat( opts, "globalMaximumHeight", hi );
		hd.clockwiseWinding = getBool( opts, "clockwiseWinding", false );
		b3HeightFieldData* data = b3CreateHeightField( &hd );
		if ( data == nullptr )
		{
			Shape shape = { b3_nullShapeId };
			return shape;
		}
		Shape shape = { b3CreateHeightFieldShape( id, &def, data ) };
		return shape;
	}
};

static void applyJointBase( b3JointDef* base, const Body& a, const Body& b, const val& opts )
{
	base->bodyIdA = a.id;
	base->bodyIdB = b.id;
	base->localFrameA = getTransform( opts, "localFrameA" );
	base->localFrameB = getTransform( opts, "localFrameB" );
	if ( hasKey( opts, "anchorA" ) )
	{
		base->localFrameA.p = getVec3( opts, "anchorA", b3Vec3_zero );
	}
	if ( hasKey( opts, "anchorB" ) )
	{
		base->localFrameB.p = getVec3( opts, "anchorB", b3Vec3_zero );
	}
	base->collideConnected = getBool( opts, "collideConnected", base->collideConnected );
	base->forceThreshold = getFloat( opts, "forceThreshold", base->forceThreshold );
	base->torqueThreshold = getFloat( opts, "torqueThreshold", base->torqueThreshold );
}

struct World
{
	b3WorldId id;
	// Set after restore(): the replay player that owns the restored world.
	b3RecPlayer* player = nullptr;

	World()
		: World( val::undefined() )
	{
	}

	explicit World( val opts )
	{
		b3WorldDef def = b3DefaultWorldDef();
		if ( !opts.isUndefined() && !opts.isNull() )
		{
			def.gravity = getVec3( opts, "gravity", def.gravity );
			def.restitutionThreshold = getFloat( opts, "restitutionThreshold", def.restitutionThreshold );
			def.hitEventThreshold = getFloat( opts, "hitEventThreshold", def.hitEventThreshold );
			def.contactHertz = getFloat( opts, "contactHertz", def.contactHertz );
			def.contactDampingRatio = getFloat( opts, "contactDampingRatio", def.contactDampingRatio );
			def.contactSpeed = getFloat( opts, "contactSpeed", def.contactSpeed );
			def.maximumLinearSpeed = getFloat( opts, "maximumLinearSpeed", def.maximumLinearSpeed );
			def.enableSleep = getBool( opts, "enableSleep", def.enableSleep );
			def.enableContinuous = getBool( opts, "enableContinuous", def.enableContinuous );
#ifdef __EMSCRIPTEN_PTHREADS__
			int workers = getInt( opts, "workerCount", 1 );
			if ( workers < 1 )
			{
				workers = 1;
			}
			if ( workers > B3_MAX_WORKERS )
			{
				workers = B3_MAX_WORKERS;
			}
			def.workerCount = (uint32_t)workers;
#endif
		}
		id = b3CreateWorld( &def );
		g_worldGen[id.index1 - 1] = id.generation;
	}

	bool isValid() const
	{
		return b3World_IsValid( id );
	}

	void destroy()
	{
		if ( player != nullptr )
		{
			b3RecPlayer_Destroy( player );
			player = nullptr;
		}
		else if ( b3World_IsValid( id ) )
		{
			b3DestroyWorld( id );
		}
		id = b3_nullWorldId;
	}

	// Snapshot. save() returns the world's state as bytes: a recording started and stopped at once holds only its
	// seed snapshot. restore(bytes) replaces this world with one restored from them, owned by a replay player, and
	// returns false (leaving the world as it was) when the bytes are refused. Body and shape ids (index and
	// generation) are the same in the restored world; handles are rebuilt with getBody. User data is not restored.
	val save() const
	{
		b3Recording* recording = b3CreateRecording( 0 );
		b3World_StartRecording( id, recording );
		b3World_StopRecording( id );
		int size = b3Recording_GetSize( recording );
		val bytes = val::global( "Uint8Array" ).new_( size );
		bytes.call<void>( "set", val( emscripten::typed_memory_view( size, b3Recording_GetData( recording ) ) ) );
		b3DestroyRecording( recording );
		return bytes;
	}

	bool restore( val bytes )
	{
		std::vector<uint8_t> data = emscripten::convertJSArrayToNumberVector<uint8_t>( bytes );
		b3RecPlayer* restored = b3RecPlayer_Create( data.data(), (int)data.size(), 1 );
		if ( restored == nullptr )
		{
			return false;
		}
		destroy();
		player = restored;
		id = b3RecPlayer_GetWorldId( restored );
		g_worldGen[id.index1 - 1] = id.generation;
		return true;
	}

	Body getBody( int index, int generation ) const
	{
		Body body;
		body.id = { index, (uint16_t)( id.index1 - 1 ), (uint16_t)generation };
		body.stamp();
		return body;
	}

	void step( float timeStep, int subStepCount )
	{
		b3World_Step( id, timeStep, subStepCount );
	}

	val getGravity() const
	{
		return fromVec3( b3World_GetGravity( id ) );
	}

	void setGravity( val gravity )
	{
		b3World_SetGravity( id, toVec3( gravity, b3Vec3_zero ) );
	}

	void enableSleeping( bool flag )
	{
		b3World_EnableSleeping( id, flag );
	}

	void enableContinuous( bool flag )
	{
		b3World_EnableContinuous( id, flag );
	}

	int getAwakeBodyCount() const
	{
		return b3World_GetAwakeBodyCount( id );
	}

	int getWorkerCount() const
	{
		return b3World_GetWorkerCount( id );
	}

	Body createBody( val opts )
	{
		b3BodyDef def = b3DefaultBodyDef();
		def.userData = (void*)(uintptr_t)( g_nextBodyTag++ );
		std::string name;
		if ( !opts.isUndefined() && !opts.isNull() )
		{
			if ( hasKey( opts, "type" ) )
			{
				std::string type = opts[K( "type" )].as<std::string>();
				if ( type == "dynamic" )
				{
					def.type = b3_dynamicBody;
				}
				else if ( type == "kinematic" )
				{
					def.type = b3_kinematicBody;
				}
				else
				{
					def.type = b3_staticBody;
				}
			}
			def.position = getVec3( opts, "position", def.position );
			def.rotation = getQuat( opts, "rotation", def.rotation );
			def.linearVelocity = getVec3( opts, "linearVelocity", def.linearVelocity );
			def.angularVelocity = getVec3( opts, "angularVelocity", def.angularVelocity );
			def.linearDamping = getFloat( opts, "linearDamping", def.linearDamping );
			def.angularDamping = getFloat( opts, "angularDamping", def.angularDamping );
			def.gravityScale = getFloat( opts, "gravityScale", def.gravityScale );
			def.sleepThreshold = getFloat( opts, "sleepThreshold", def.sleepThreshold );
			def.enableSleep = getBool( opts, "enableSleep", def.enableSleep );
			def.isAwake = getBool( opts, "isAwake", def.isAwake );
			def.isBullet = getBool( opts, "isBullet", def.isBullet );
			def.isEnabled = getBool( opts, "isEnabled", def.isEnabled );
			def.allowFastRotation = getBool( opts, "allowFastRotation", def.allowFastRotation );
			def.enableContactRecycling = getBool( opts, "enableContactRecycling", def.enableContactRecycling );
			if ( hasKey( opts, "motionLocks" ) )
			{
				val locks = opts[K( "motionLocks" )];
				def.motionLocks.linearX = getBool( locks, "linearX", false );
				def.motionLocks.linearY = getBool( locks, "linearY", false );
				def.motionLocks.linearZ = getBool( locks, "linearZ", false );
				def.motionLocks.angularX = getBool( locks, "angularX", false );
				def.motionLocks.angularY = getBool( locks, "angularY", false );
				def.motionLocks.angularZ = getBool( locks, "angularZ", false );
			}
			if ( hasKey( opts, "userData" ) )
			{
				def.userData = (void*)(uintptr_t)opts[K( "userData" )].as<double>();
			}
			if ( hasKey( opts, "name" ) )
			{
				name = opts[K( "name" )].as<std::string>();
				def.name = name.c_str();
			}
		}
		Body body = { b3CreateBody( id, &def ) };
		return body;
	}

	// Joints

	DistanceJoint createDistanceJoint( const Body& a, const Body& b, val opts )
	{
		b3DistanceJointDef def = b3DefaultDistanceJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.length = getFloat( opts, "length", def.length );
		def.enableSpring = getBool( opts, "enableSpring", def.enableSpring );
		def.hertz = getFloat( opts, "hertz", def.hertz );
		def.dampingRatio = getFloat( opts, "dampingRatio", def.dampingRatio );
		def.lowerSpringForce = getFloat( opts, "lowerSpringForce", def.lowerSpringForce );
		def.upperSpringForce = getFloat( opts, "upperSpringForce", def.upperSpringForce );
		def.enableLimit = getBool( opts, "enableLimit", def.enableLimit );
		def.minLength = getFloat( opts, "minLength", def.minLength );
		def.maxLength = getFloat( opts, "maxLength", def.maxLength );
		def.enableMotor = getBool( opts, "enableMotor", def.enableMotor );
		def.maxMotorForce = getFloat( opts, "maxMotorForce", def.maxMotorForce );
		def.motorSpeed = getFloat( opts, "motorSpeed", def.motorSpeed );
		DistanceJoint joint;
		joint.id = b3CreateDistanceJoint( id, &def );
		joint.stamp();
		return joint;
	}

	RevoluteJoint createRevoluteJoint( const Body& a, const Body& b, val opts )
	{
		b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.targetAngle = getFloat( opts, "targetAngle", def.targetAngle );
		def.enableSpring = getBool( opts, "enableSpring", def.enableSpring );
		def.hertz = getFloat( opts, "hertz", def.hertz );
		def.dampingRatio = getFloat( opts, "dampingRatio", def.dampingRatio );
		def.enableLimit = getBool( opts, "enableLimit", def.enableLimit );
		def.lowerAngle = getFloat( opts, "lowerAngle", def.lowerAngle );
		def.upperAngle = getFloat( opts, "upperAngle", def.upperAngle );
		def.enableMotor = getBool( opts, "enableMotor", def.enableMotor );
		def.maxMotorTorque = getFloat( opts, "maxMotorTorque", def.maxMotorTorque );
		def.motorSpeed = getFloat( opts, "motorSpeed", def.motorSpeed );
		RevoluteJoint joint;
		joint.id = b3CreateRevoluteJoint( id, &def );
		joint.stamp();
		return joint;
	}

	SphericalJoint createSphericalJoint( const Body& a, const Body& b, val opts )
	{
		b3SphericalJointDef def = b3DefaultSphericalJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.enableSpring = getBool( opts, "enableSpring", def.enableSpring );
		def.hertz = getFloat( opts, "hertz", def.hertz );
		def.dampingRatio = getFloat( opts, "dampingRatio", def.dampingRatio );
		def.targetRotation = getQuat( opts, "targetRotation", def.targetRotation );
		def.enableConeLimit = getBool( opts, "enableConeLimit", def.enableConeLimit );
		def.coneAngle = getFloat( opts, "coneAngle", def.coneAngle );
		def.enableTwistLimit = getBool( opts, "enableTwistLimit", def.enableTwistLimit );
		def.lowerTwistAngle = getFloat( opts, "lowerTwistAngle", def.lowerTwistAngle );
		def.upperTwistAngle = getFloat( opts, "upperTwistAngle", def.upperTwistAngle );
		def.enableMotor = getBool( opts, "enableMotor", def.enableMotor );
		def.maxMotorTorque = getFloat( opts, "maxMotorTorque", def.maxMotorTorque );
		def.motorVelocity = getVec3( opts, "motorVelocity", def.motorVelocity );
		SphericalJoint joint;
		joint.id = b3CreateSphericalJoint( id, &def );
		joint.stamp();
		return joint;
	}

	PrismaticJoint createPrismaticJoint( const Body& a, const Body& b, val opts )
	{
		b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.enableSpring = getBool( opts, "enableSpring", def.enableSpring );
		def.hertz = getFloat( opts, "hertz", def.hertz );
		def.dampingRatio = getFloat( opts, "dampingRatio", def.dampingRatio );
		def.targetTranslation = getFloat( opts, "targetTranslation", def.targetTranslation );
		def.enableLimit = getBool( opts, "enableLimit", def.enableLimit );
		def.lowerTranslation = getFloat( opts, "lowerTranslation", def.lowerTranslation );
		def.upperTranslation = getFloat( opts, "upperTranslation", def.upperTranslation );
		def.enableMotor = getBool( opts, "enableMotor", def.enableMotor );
		def.maxMotorForce = getFloat( opts, "maxMotorForce", def.maxMotorForce );
		def.motorSpeed = getFloat( opts, "motorSpeed", def.motorSpeed );
		PrismaticJoint joint;
		joint.id = b3CreatePrismaticJoint( id, &def );
		joint.stamp();
		return joint;
	}

	WeldJoint createWeldJoint( const Body& a, const Body& b, val opts )
	{
		b3WeldJointDef def = b3DefaultWeldJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.linearHertz = getFloat( opts, "linearHertz", def.linearHertz );
		def.angularHertz = getFloat( opts, "angularHertz", def.angularHertz );
		def.linearDampingRatio = getFloat( opts, "linearDampingRatio", def.linearDampingRatio );
		def.angularDampingRatio = getFloat( opts, "angularDampingRatio", def.angularDampingRatio );
		WeldJoint joint;
		joint.id = b3CreateWeldJoint( id, &def );
		joint.stamp();
		return joint;
	}

	MotorJoint createMotorJoint( const Body& a, const Body& b, val opts )
	{
		b3MotorJointDef def = b3DefaultMotorJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.linearVelocity = getVec3( opts, "linearVelocity", def.linearVelocity );
		def.maxVelocityForce = getFloat( opts, "maxVelocityForce", def.maxVelocityForce );
		def.angularVelocity = getVec3( opts, "angularVelocity", def.angularVelocity );
		def.maxVelocityTorque = getFloat( opts, "maxVelocityTorque", def.maxVelocityTorque );
		def.linearHertz = getFloat( opts, "linearHertz", def.linearHertz );
		def.linearDampingRatio = getFloat( opts, "linearDampingRatio", def.linearDampingRatio );
		def.maxSpringForce = getFloat( opts, "maxSpringForce", def.maxSpringForce );
		def.angularHertz = getFloat( opts, "angularHertz", def.angularHertz );
		def.angularDampingRatio = getFloat( opts, "angularDampingRatio", def.angularDampingRatio );
		def.maxSpringTorque = getFloat( opts, "maxSpringTorque", def.maxSpringTorque );
		MotorJoint joint;
		joint.id = b3CreateMotorJoint( id, &def );
		joint.stamp();
		return joint;
	}

	WheelJoint createWheelJoint( const Body& a, const Body& b, val opts )
	{
		b3WheelJointDef def = b3DefaultWheelJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.enableSuspensionSpring = getBool( opts, "enableSuspensionSpring", def.enableSuspensionSpring );
		def.suspensionHertz = getFloat( opts, "suspensionHertz", def.suspensionHertz );
		def.suspensionDampingRatio = getFloat( opts, "suspensionDampingRatio", def.suspensionDampingRatio );
		def.enableSuspensionLimit = getBool( opts, "enableSuspensionLimit", def.enableSuspensionLimit );
		def.lowerSuspensionLimit = getFloat( opts, "lowerSuspensionLimit", def.lowerSuspensionLimit );
		def.upperSuspensionLimit = getFloat( opts, "upperSuspensionLimit", def.upperSuspensionLimit );
		def.enableSpinMotor = getBool( opts, "enableSpinMotor", def.enableSpinMotor );
		def.maxSpinTorque = getFloat( opts, "maxSpinTorque", def.maxSpinTorque );
		def.spinSpeed = getFloat( opts, "spinSpeed", def.spinSpeed );
		def.enableSteering = getBool( opts, "enableSteering", def.enableSteering );
		def.steeringHertz = getFloat( opts, "steeringHertz", def.steeringHertz );
		def.steeringDampingRatio = getFloat( opts, "steeringDampingRatio", def.steeringDampingRatio );
		def.targetSteeringAngle = getFloat( opts, "targetSteeringAngle", def.targetSteeringAngle );
		def.maxSteeringTorque = getFloat( opts, "maxSteeringTorque", def.maxSteeringTorque );
		def.enableSteeringLimit = getBool( opts, "enableSteeringLimit", def.enableSteeringLimit );
		def.lowerSteeringLimit = getFloat( opts, "lowerSteeringLimit", def.lowerSteeringLimit );
		def.upperSteeringLimit = getFloat( opts, "upperSteeringLimit", def.upperSteeringLimit );
		WheelJoint joint;
		joint.id = b3CreateWheelJoint( id, &def );
		joint.stamp();
		return joint;
	}

	ParallelJoint createParallelJoint( const Body& a, const Body& b, val opts )
	{
		b3ParallelJointDef def = b3DefaultParallelJointDef();
		applyJointBase( &def.base, a, b, opts );
		def.hertz = getFloat( opts, "hertz", def.hertz );
		def.dampingRatio = getFloat( opts, "dampingRatio", def.dampingRatio );
		def.maxTorque = getFloat( opts, "maxTorque", def.maxTorque );
		ParallelJoint joint;
		joint.id = b3CreateParallelJoint( id, &def );
		joint.stamp();
		return joint;
	}

	FilterJoint createFilterJoint( const Body& a, const Body& b, val opts )
	{
		b3FilterJointDef def = b3DefaultFilterJointDef();
		applyJointBase( &def.base, a, b, opts );
		FilterJoint joint;
		joint.id = b3CreateFilterJoint( id, &def );
		joint.stamp();
		return joint;
	}

	// Queries

	val castRayClosest( val origin, val translation, val filterOpts ) const
	{
		b3RayResult result = b3World_CastRayClosest( id, toVec3( origin, b3Vec3_zero ), toVec3( translation, b3Vec3_zero ),
													 queryFilterFromOpts( filterOpts ) );
		val o = val::object();
		o.set( K( "hit" ), result.hit );
		if ( result.hit )
		{
			o.set( K( "point" ), fromVec3( result.point ) );
			o.set( K( "normal" ), fromVec3( result.normal ) );
			o.set( K( "fraction" ), result.fraction );
			o.set( K( "shapeUserData" ), tagOf( b3Shape_GetUserData( result.shapeId ) ) );
			o.set( K( "bodyUserData" ), tagOf( b3Body_GetUserData( b3Shape_GetBody( result.shapeId ) ) ) );
			Shape shape = { result.shapeId };
			o.set( K( "shape" ), val( shape ) );
		}
		return o;
	}

	// The closest ray hit (b3World_CastRay with a clipping callback), sensors left out unless
	// filterOpts.includeSensors. Unlike castRayClosest, the hit names its body by index too.
	val castRay( val origin, val translation, val filterOpts ) const
	{
		ClosestCastContext c = { includeSensorsOf( filterOpts ), false, b3_nullShapeId, b3Vec3_zero, b3Vec3_zero, 1.0f };
		b3World_CastRay( id, toVec3( origin, b3Vec3_zero ), toVec3( translation, b3Vec3_zero ), queryFilterFromOpts( filterOpts ),
						 closestCastFcn, &c );
		return closestCastResult( c );
	}

	// The closest hit of a shape proxy ({ points, radius }, relative to origin) swept by translation
	// (b3World_CastShape). Shapes the proxy overlaps at the start are reported at fraction 0.
	val castShape( val origin, val proxyOpts, val translation, val filterOpts ) const
	{
		b3Vec3 points[B3_MAX_SHAPE_CAST_POINTS];
		b3ShapeProxy proxy = proxyFromOpts( proxyOpts, points );
		ClosestCastContext c = { includeSensorsOf( filterOpts ), false, b3_nullShapeId, b3Vec3_zero, b3Vec3_zero, 1.0f };
		b3World_CastShape( id, toVec3( origin, b3Vec3_zero ), &proxy, toVec3( translation, b3Vec3_zero ),
						   queryFilterFromOpts( filterOpts ), closestCastFcn, &c );
		return closestCastResult( c );
	}

	// Every shape a proxy ({ points, radius }, relative to origin) overlaps (b3World_OverlapShape): an array of
	// { bodyIndex, bodyUserData, shapeUserData }, one per shape, in the tree's order.
	val overlapShape( val origin, val proxyOpts, val filterOpts ) const
	{
		b3Vec3 points[B3_MAX_SHAPE_CAST_POINTS];
		b3ShapeProxy proxy = proxyFromOpts( proxyOpts, points );
		OverlapContext c = { includeSensorsOf( filterOpts ), {} };
		b3World_OverlapShape( id, toVec3( origin, b3Vec3_zero ), &proxy, queryFilterFromOpts( filterOpts ), overlapFcn, &c );
		val out = val::array();
		for ( size_t i = 0; i < c.shapes.size(); ++i )
		{
			val o = val::object();
			setHitShape( o, c.shapes[i] );
			out.set( i, o );
		}
		return out;
	}

	// The nearest point on any convex shape within maxDistance of target (closestPointOf: meshes, height fields and
	// compounds are left out). { hit, point, distance, bodyIndex, bodyUserData, shapeUserData }; ties go to the shape
	// found first.
	val closestPoint( val target, float maxDistance, val filterOpts ) const
	{
		ClosestPointContext c = closestPointOf( id, toVec3( target, b3Vec3_zero ), maxDistance, queryFilterFromOpts( filterOpts ),
												includeSensorsOf( filterOpts ) );
		val o = val::object();
		o.set( K( "hit" ), c.hit );
		if ( c.hit )
		{
			o.set( K( "point" ), fromVec3( c.point ) );
			o.set( K( "distance" ), c.best );
			setHitShape( o, c.shapeId );
		}
		return o;
	}

	// `count` queries in one call, without JS objects (see "Flat calls"). A query is 12 words at flatIn:
	//   0 kind: 0 ray, 1 shape cast, 2 overlap, 3 closest point
	//   1 flags: bits 0-1 the shape (0 none, 1 sphere: one point, 2 capsule: two points along y), bit 2 include
	//     sensors, bit 3 word 11 holds the mask bits (else the default filter)
	//   2 radius, 3 the capsule's half height (floats)
	//   4-6 the origin (ray, cast, overlap) or the target (closest point), 7-9 the translation (ray, cast), floats
	//   10 the maximum distance (closest point), float; 11 the mask bits, the low 32 (unsigned)
	// The results follow each other at flatOut:
	//   ray, cast: 7 words: hit (0 or 1), fraction, normal (3 floats), the body's index and generation
	//   closest point: 7 words: hit, distance, point (3 floats), the body's index and generation
	//   overlap: 1 + 2n words: n, then the index and generation of each shape's body (one entry per shape)
	// Returns the words written; a query of an unknown kind ends the batch there.
	int queryBatch( int count ) const
	{
		g_flatOut.clear();
		// Divided, not multiplied: 12 * count wraps for a count near 2^32 / 12 and let the loop read past the input.
		if ( count < 0 || (size_t)count > g_flatIn.size() / 12 )
		{
			return 0;
		}
		for ( int i = 0; i < count; ++i )
		{
			const FlatWord* q = g_flatIn.data() + 12 * i;
			uint32_t kind = q[0].u, flags = q[1].u;
			bool includeSensors = ( flags & 4 ) != 0;
			b3QueryFilter filter = b3DefaultQueryFilter();
			if ( flags & 8 )
			{
				filter.maskBits = q[11].u;
			}
			b3Vec3 from = { q[4].f, q[5].f, q[6].f };
			b3Vec3 along = { q[7].f, q[8].f, q[9].f };
			b3Vec3 points[2] = { b3Vec3_zero, b3Vec3_zero };
			b3ShapeProxy proxy = { points, 1, q[2].f };
			if ( ( flags & 3 ) == 2 )
			{
				points[0].y = -q[3].f;
				points[1].y = q[3].f;
				proxy.count = 2;
			}
			if ( kind == 0 || kind == 1 )
			{
				ClosestCastContext c = { includeSensors, false, b3_nullShapeId, b3Vec3_zero, b3Vec3_zero, 1.0f };
				if ( kind == 0 )
				{
					b3World_CastRay( id, from, along, filter, closestCastFcn, &c );
				}
				else
				{
					b3World_CastShape( id, from, &proxy, along, filter, closestCastFcn, &c );
				}
				putU( c.hit ? 1 : 0 );
				putF( c.hit ? c.fraction : 1.0f );
				putF( c.normal.x );
				putF( c.normal.y );
				putF( c.normal.z );
				putShapeBody( c.shapeId );
			}
			else if ( kind == 2 )
			{
				OverlapContext c = { includeSensors, {} };
				b3World_OverlapShape( id, from, &proxy, filter, overlapFcn, &c );
				putU( (uint32_t)c.shapes.size() );
				for ( b3ShapeId s : c.shapes )
				{
					putBody( b3Shape_GetBody( s ) );
				}
			}
			else if ( kind == 3 )
			{
				ClosestPointContext c = closestPointOf( id, from, q[10].f, filter, includeSensors );
				putU( c.hit ? 1 : 0 );
				putF( c.best );
				putF( c.point.x );
				putF( c.point.y );
				putF( c.point.z );
				putShapeBody( c.shapeId );
			}
			else
			{
				break;
			}
		}
		return (int)g_flatOut.size();
	}

	// The bodies that moved in the last step (b3World_GetBodyEvents) with their velocities, 16 words each at flatOut:
	// index, generation, position (3 floats), rotation (x y z w), linear velocity (3), angular velocity (3), fell
	// asleep (0 or 1). A body destroyed since the step is left out. Returns the bodies written.
	int readMoved() const
	{
		b3BodyEvents events = b3World_GetBodyEvents( id );
		// Sized once and written through a pointer: at -Os a push_back per word is not inlined, on the hottest read path.
		g_flatOut.resize( (size_t)events.moveCount * 16 );
		FlatWord* out = g_flatOut.data();
		int written = 0;
		for ( int i = 0; i < events.moveCount; ++i )
		{
			const b3BodyMoveEvent& e = events.moveEvents[i];
			// Its velocities come from the body: one destroyed after the step (or its slot reused) has none to give.
			if ( !b3Body_IsValid( e.bodyId ) )
			{
				continue;
			}
			++written;
			b3Vec3 v = b3Body_GetLinearVelocity( e.bodyId ), w = b3Body_GetAngularVelocity( e.bodyId );
			out[0].u = (uint32_t)e.bodyId.index1;
			out[1].u = e.bodyId.generation;
			out[2].f = (float)e.transform.p.x;
			out[3].f = (float)e.transform.p.y;
			out[4].f = (float)e.transform.p.z;
			out[5].f = e.transform.q.v.x;
			out[6].f = e.transform.q.v.y;
			out[7].f = e.transform.q.v.z;
			out[8].f = e.transform.q.s;
			out[9].f = v.x;
			out[10].f = v.y;
			out[11].f = v.z;
			out[12].f = w.x;
			out[13].f = w.y;
			out[14].f = w.z;
			out[15].u = e.fellAsleep ? 1 : 0;
			out += 16;
		}
		g_flatOut.resize( (size_t)written * 16 );
		return written;
	}

	// The contact and sensor events of the last step, 10 words each at flatOut: kind (1 contact begin, 2 contact
	// end, 3 sensor begin, 4 sensor end), shape A's index and B's (a sensor event: the sensor, then the visitor), the
	// index and generation of body A and of body B (both all ones when the shape was destroyed), the normal from A to B
	// (3 floats: a contact begin's first manifold; zero otherwise). Returns the events.
	int readEvents() const
	{
		g_flatOut.clear();
		int n = 0;
		auto put = [&]( uint32_t kind, b3ShapeId a, b3ShapeId b, b3Vec3 normal ) {
			putU( kind );
			putU( (uint32_t)a.index1 );
			putU( (uint32_t)b.index1 );
			putShapeBody( a );
			putShapeBody( b );
			putF( normal.x );
			putF( normal.y );
			putF( normal.z );
			++n;
		};
		b3ContactEvents contacts = b3World_GetContactEvents( id );
		for ( int i = 0; i < contacts.beginCount; ++i )
		{
			const b3ContactBeginTouchEvent& e = contacts.beginEvents[i];
			b3Vec3 normal = b3Vec3_zero;
			if ( b3Contact_IsValid( e.contactId ) )
			{
				b3ContactData data = b3Contact_GetData( e.contactId );
				if ( data.manifoldCount > 0 )
				{
					normal = data.manifolds[0].normal;
				}
			}
			put( 1, e.shapeIdA, e.shapeIdB, normal );
		}
		for ( int i = 0; i < contacts.endCount; ++i )
		{
			put( 2, contacts.endEvents[i].shapeIdA, contacts.endEvents[i].shapeIdB, b3Vec3_zero );
		}
		b3SensorEvents sensors = b3World_GetSensorEvents( id );
		for ( int i = 0; i < sensors.beginCount; ++i )
		{
			put( 3, sensors.beginEvents[i].sensorShapeId, sensors.beginEvents[i].visitorShapeId, b3Vec3_zero );
		}
		for ( int i = 0; i < sensors.endCount; ++i )
		{
			put( 4, sensors.endEvents[i].sensorShapeId, sensors.endEvents[i].visitorShapeId, b3Vec3_zero );
		}
		return n;
	}

	// `count` body writes in one call, 12 words each at flatIn: op, the body's index and generation, then up to 9
	// floats. Ops: 0 linear velocity (x y z), 1 angular velocity (x y z), 2 transform (position, rotation x y z w;
	// b3Body_SetTransform), 3 target transform (position, rotation, time step; wakes the body), 4 linear impulse at
	// the center (x y z; wakes), 5 angular impulse (x y z; wakes). A record whose body is not valid (destroyed, or an
	// old generation) or whose op is unknown is skipped. Returns the records applied.
	int writeBodies( int count ) const
	{
		// Divided, not multiplied: 12 * count wraps for a count near 2^32 / 12 and let the loop read past the input.
		if ( count < 0 || (size_t)count > g_flatIn.size() / 12 )
		{
			return 0;
		}
		int applied = 0;
		for ( int i = 0; i < count; ++i )
		{
			const FlatWord* r = g_flatIn.data() + 12 * i;
			b3BodyId body = { (int32_t)r[1].u, (uint16_t)( id.index1 - 1 ), (uint16_t)r[2].u };
			if ( r[0].u > 5 || r[2].u > UINT16_MAX || !b3Body_IsValid( body ) )
			{
				continue;
			}
			b3Vec3 v = { r[3].f, r[4].f, r[5].f };
			b3Quat q = { { r[6].f, r[7].f, r[8].f }, r[9].f };
			switch ( r[0].u )
			{
				case 0:
					b3Body_SetLinearVelocity( body, v );
					break;
				case 1:
					b3Body_SetAngularVelocity( body, v );
					break;
				case 2:
					b3Body_SetTransform( body, v, q );
					break;
				case 3:
				{
					b3WorldTransform xf;
					xf.p = v;
					xf.q = q;
					b3Body_SetTargetTransform( body, xf, r[10].f, true );
					break;
				}
				case 4:
					b3Body_ApplyLinearImpulseToCenter( body, v, true );
					break;
				default:
					b3Body_ApplyAngularImpulse( body, v, true );
					break;
			}
			++applied;
		}
		return applied;
	}

	// The fraction of translation a capsule mover ({ center1, center2, radius }, relative to origin) can travel
	// (b3World_CastMover). Shapes it already overlaps are ignored, as are sensors unless filterOpts.includeSensors.
	float castMover( val origin, val capsuleOpts, val translation, val filterOpts ) const
	{
		b3Capsule mover = capsuleFromOpts( capsuleOpts );
		bool includeSensors = includeSensorsOf( filterOpts );
		return b3World_CastMover( id, toVec3( origin, b3Vec3_zero ), &mover, toVec3( translation, b3Vec3_zero ),
								  queryFilterFromOpts( filterOpts ), moverFilterFcn, &includeSensors );
	}

	// The collision planes of a capsule mover (b3World_CollideMover), relative to origin: an array of
	// { normal, offset, point, bodyIndex, bodyUserData, shapeUserData }, for solvePlanes.
	val collideMover( val origin, val capsuleOpts, val filterOpts ) const
	{
		b3Capsule mover = capsuleFromOpts( capsuleOpts );
		PlaneContext c = { includeSensorsOf( filterOpts ), val::array(), 0 };
		b3World_CollideMover( id, toVec3( origin, b3Vec3_zero ), &mover, queryFilterFromOpts( filterOpts ), planeFcn, &c );
		return c.out;
	}

	// Mover calls without JS objects (see "Flat calls"), 20 words each at flatIn:
	//   0 kind: 0 move (b3World_CollideMover, b3SolvePlanes, b3World_CastMover in rounds, as Box2D's character sample
	//     loops them), 1 cast (b3World_CastMover), 2 ground (of the planes the capsule touches, the first one whose
	//     normal points most up, at or above word 15)
	//   1 flags: bit 0 a move lays each plane whose normal y is below word 15 flat (a wall: the normal's horizontal part
	//     at unit length, the offset kept), bit 1 include sensors, bit 2 word 16 holds the mask bits
	//   2-4 the capsule's position, 5-7 and 8-10 its centers (relative to the position), 11 its radius (floats)
	//   12-14 the translation (move, cast), 15 the least normal y (move with bit 0, ground), floats
	//   16 the mask bits, the low 32; 17 a move's most rounds (0: 5; at most 64); 18, 19 unused
	// A move aims at the position plus the translation; each round solves the planes for what is left, casts that
	// and moves by the fraction cast; it stops when a round moves less than 1e-4. The positions are kept in doubles
	// between rounds, as a JS loop over the object calls keeps them; the inputs and outputs are float32 words, so a
	// caller that keeps positions in doubles rounds them once a call, and the least normal y (word 15) is a float32:
	// compare against Math.fround(Math.cos(slope)) in a JS path that must agree with this one.
	// Results, 4 words each at flatOut: move: the end position (3 floats) and the rounds run; cast: the fraction, then
	// zeros; ground: found (0 or 1), the body's index and generation, the normal's y (float). Returns the records
	// answered; a record of an unknown kind ends the batch there.
	int moverBatch( int count ) const
	{
		g_flatOut.clear();
		if ( count < 0 || (size_t)count > g_flatIn.size() / 20 )
		{
			return 0;
		}
		int answered = 0;
		std::vector<b3CollisionPlane> planes;
		for ( int i = 0; i < count; ++i )
		{
			const FlatWord* r = g_flatIn.data() + 20 * i;
			uint32_t kind = r[0].u, flags = r[1].u;
			bool includeSensors = ( flags & 2 ) != 0;
			b3QueryFilter filter = b3DefaultQueryFilter();
			if ( flags & 4 )
			{
				filter.maskBits = r[16].u;
			}
			b3Capsule capsule = { { r[5].f, r[6].f, r[7].f }, { r[8].f, r[9].f, r[10].f }, r[11].f };
			b3Vec3 at = { r[2].f, r[3].f, r[4].f }, along = { r[12].f, r[13].f, r[14].f };
			float minY = r[15].f;
			if ( kind == 0 )
			{
				// At most 64 rounds: a word of 2^32 - 1 would hold the call for minutes (review of 2026-11-03).
				int rounds = r[17].u == 0 ? 5 : r[17].u > 64 ? 64 : (int)r[17].u, run = 0;
				double px = at.x, py = at.y, pz = at.z;
				double tx = px + along.x, ty = py + along.y, tz = pz + along.z;
				while ( run < rounds )
				{
					++run;
					b3Vec3 p = { (float)px, (float)py, (float)pz };
					PlaneList found = { includeSensors, {}, {} };
					b3World_CollideMover( id, p, &capsule, filter, planeListFcn, &found );
					planes.clear();
					for ( const b3PlaneResult& pr : found.planes )
					{
						b3CollisionPlane cp = { pr.plane, FLT_MAX, 0.0f, true };
						double h = std::hypot( (double)pr.plane.normal.x, (double)pr.plane.normal.z );
						if ( ( flags & 1 ) && pr.plane.normal.y < minY && h > 1e-6 )
						{
							cp.plane.normal = { (float)( pr.plane.normal.x / h ), 0.0f, (float)( pr.plane.normal.z / h ) };
						}
						planes.push_back( cp );
					}
					b3Vec3 left = { (float)( tx - px ), (float)( ty - py ), (float)( tz - pz ) };
					b3Vec3 e = b3SolvePlanes( left, planes.data(), (int)planes.size() ).delta;
					float f = b3World_CastMover( id, p, &capsule, e, filter, moverFilterFcn, &includeSensors );
					double mx = (double)f * e.x, my = (double)f * e.y, mz = (double)f * e.z;
					px += mx;
					py += my;
					pz += mz;
					if ( std::sqrt( mx * mx + my * my + mz * mz ) < 1e-4 )
					{
						break;
					}
				}
				putF( (float)px );
				putF( (float)py );
				putF( (float)pz );
				putU( (uint32_t)run );
			}
			else if ( kind == 1 )
			{
				putF( b3World_CastMover( id, at, &capsule, along, filter, moverFilterFcn, &includeSensors ) );
				putU( 0 );
				putU( 0 );
				putU( 0 );
			}
			else if ( kind == 2 )
			{
				PlaneList found = { includeSensors, {}, {} };
				b3World_CollideMover( id, at, &capsule, filter, planeListFcn, &found );
				int best = -1;
				for ( int k = 0; k < (int)found.planes.size(); ++k )
				{
					float y = found.planes[k].plane.normal.y;
					if ( y >= minY && ( best < 0 || y > found.planes[best].plane.normal.y ) )
					{
						best = k;
					}
				}
				putU( best >= 0 ? 1 : 0 );
				putShapeBody( best >= 0 ? found.shapes[best] : b3_nullShapeId );
				putF( best >= 0 ? found.planes[best].plane.normal.y : 0.0f );
			}
			else
			{
				break;
			}
			++answered;
		}
		return answered;
	}

	void explode( val opts )
	{
		b3ExplosionDef def = b3DefaultExplosionDef();
		def.position = getVec3( opts, "position", def.position );
		def.radius = getFloat( opts, "radius", def.radius );
		def.falloff = getFloat( opts, "falloff", def.falloff );
		def.impulsePerArea = getFloat( opts, "impulsePerArea", def.impulsePerArea );
		def.maskBits = getU64( opts, "maskBits", def.maskBits );
		b3World_Explode( id, &def );
	}

	// Events. Arrays of plain objects, correlate with userData tags.

	val getBodyEvents() const
	{
		b3BodyEvents events = b3World_GetBodyEvents( id );
		val arr = val::array();
		for ( int i = 0; i < events.moveCount; ++i )
		{
			const b3BodyMoveEvent& e = events.moveEvents[i];
			val o = val::object();
			o.set( K( "userData" ), tagOf( e.userData ) );
			o.set( K( "position" ), fromVec3( e.transform.p ) );
			o.set( K( "rotation" ), fromQuat( e.transform.q ) );
			o.set( K( "fellAsleep" ), e.fellAsleep );
			arr.set( i, o );
		}
		return arr;
	}

	// The body move events as typed arrays, not one object per event: { count, bodyIndex (Uint32Array),
	// userData (Uint32Array), transform (Float32Array, x y z qx qy qz qw per event), fellAsleep (Uint8Array) }.
	// The arrays are views of the module's memory, valid until the next call into the module.
	val getBodyMoveEvents() const
	{
		static std::vector<uint32_t> index, tags;
		static std::vector<float> transform;
		static std::vector<uint8_t> asleep;
		b3BodyEvents events = b3World_GetBodyEvents( id );
		int n = events.moveCount;
		index.resize( n );
		tags.resize( n );
		transform.resize( 7 * n );
		asleep.resize( n );
		for ( int i = 0; i < n; ++i )
		{
			const b3BodyMoveEvent& e = events.moveEvents[i];
			index[i] = (uint32_t)e.bodyId.index1;
			tags[i] = (uint32_t)(uintptr_t)e.userData;
			float* t = transform.data() + 7 * i;
			t[0] = (float)e.transform.p.x;
			t[1] = (float)e.transform.p.y;
			t[2] = (float)e.transform.p.z;
			t[3] = e.transform.q.v.x;
			t[4] = e.transform.q.v.y;
			t[5] = e.transform.q.v.z;
			t[6] = e.transform.q.s;
			asleep[i] = e.fellAsleep ? 1 : 0;
		}
		val o = val::object();
		o.set( K( "count" ), n );
		o.set( K( "bodyIndex" ), val( emscripten::typed_memory_view( index.size(), index.data() ) ) );
		o.set( K( "userData" ), val( emscripten::typed_memory_view( tags.size(), tags.data() ) ) );
		o.set( K( "transform" ), val( emscripten::typed_memory_view( transform.size(), transform.data() ) ) );
		o.set( K( "fellAsleep" ), val( emscripten::typed_memory_view( asleep.size(), asleep.data() ) ) );
		return o;
	}

	val getContactEvents() const
	{
		b3ContactEvents events = b3World_GetContactEvents( id );
		val out = val::object();

		val begin = val::array();
		for ( int i = 0; i < events.beginCount; ++i )
		{
			const b3ContactBeginTouchEvent& e = events.beginEvents[i];
			val o = val::object();
			o.set( K( "shapeUserDataA" ), tagOf( b3Shape_GetUserData( e.shapeIdA ) ) );
			o.set( K( "shapeUserDataB" ), tagOf( b3Shape_GetUserData( e.shapeIdB ) ) );
			begin.set( i, o );
		}
		out.set( K( "begin" ), begin );

		val end = val::array();
		for ( int i = 0; i < events.endCount; ++i )
		{
			const b3ContactEndTouchEvent& e = events.endEvents[i];
			val o = val::object();
			bool validA = b3Shape_IsValid( e.shapeIdA );
			bool validB = b3Shape_IsValid( e.shapeIdB );
			o.set( K( "shapeUserDataA" ), validA ? val( tagOf( b3Shape_GetUserData( e.shapeIdA ) ) ) : val::null() );
			o.set( K( "shapeUserDataB" ), validB ? val( tagOf( b3Shape_GetUserData( e.shapeIdB ) ) ) : val::null() );
			end.set( i, o );
		}
		out.set( K( "end" ), end );

		val hit = val::array();
		for ( int i = 0; i < events.hitCount; ++i )
		{
			const b3ContactHitEvent& e = events.hitEvents[i];
			val o = val::object();
			o.set( K( "shapeUserDataA" ), tagOf( b3Shape_GetUserData( e.shapeIdA ) ) );
			o.set( K( "shapeUserDataB" ), tagOf( b3Shape_GetUserData( e.shapeIdB ) ) );
			o.set( K( "point" ), fromVec3( e.point ) );
			o.set( K( "normal" ), fromVec3( e.normal ) );
			o.set( K( "approachSpeed" ), e.approachSpeed );
			hit.set( i, o );
		}
		out.set( K( "hit" ), hit );

		return out;
	}

	val getSensorEvents() const
	{
		b3SensorEvents events = b3World_GetSensorEvents( id );
		val out = val::object();

		val begin = val::array();
		for ( int i = 0; i < events.beginCount; ++i )
		{
			val o = val::object();
			o.set( K( "sensorUserData" ), tagOf( b3Shape_GetUserData( events.beginEvents[i].sensorShapeId ) ) );
			o.set( K( "visitorUserData" ), tagOf( b3Shape_GetUserData( events.beginEvents[i].visitorShapeId ) ) );
			begin.set( i, o );
		}
		out.set( K( "begin" ), begin );

		val end = val::array();
		for ( int i = 0; i < events.endCount; ++i )
		{
			val o = val::object();
			bool validSensor = b3Shape_IsValid( events.endEvents[i].sensorShapeId );
			bool validVisitor = b3Shape_IsValid( events.endEvents[i].visitorShapeId );
			o.set( K( "sensorUserData" ),
				   validSensor ? val( tagOf( b3Shape_GetUserData( events.endEvents[i].sensorShapeId ) ) ) : val::null() );
			o.set( K( "visitorUserData" ),
				   validVisitor ? val( tagOf( b3Shape_GetUserData( events.endEvents[i].visitorShapeId ) ) ) : val::null() );
			end.set( i, o );
		}
		out.set( K( "end" ), end );

		return out;
	}

	// The joints whose force or torque exceeded their threshold in the last step: an array of { jointIndex, userData }.
	val getJointEvents() const
	{
		b3JointEvents events = b3World_GetJointEvents( id );
		val out = val::array();
		for ( int i = 0; i < events.count; ++i )
		{
			val o = val::object();
			o.set( K( "jointIndex" ), events.jointEvents[i].jointId.index1 );
			o.set( K( "userData" ), tagOf( events.jointEvents[i].userData ) );
			out.set( i, o );
		}
		return out;
	}

	val getProfile() const
	{
		b3Profile p = b3World_GetProfile( id );
		val o = val::object();
		o.set( K( "step" ), p.step );
		o.set( K( "pairs" ), p.pairs );
		o.set( K( "collide" ), p.collide );
		o.set( K( "solve" ), p.solve );
		o.set( K( "solverSetup" ), p.solverSetup );
		o.set( K( "constraints" ), p.constraints );
		o.set( K( "prepareConstraints" ), p.prepareConstraints );
		o.set( K( "integrateVelocities" ), p.integrateVelocities );
		o.set( K( "warmStart" ), p.warmStart );
		o.set( K( "solveImpulses" ), p.solveImpulses );
		o.set( K( "integratePositions" ), p.integratePositions );
		o.set( K( "relaxImpulses" ), p.relaxImpulses );
		o.set( K( "applyRestitution" ), p.applyRestitution );
		o.set( K( "storeImpulses" ), p.storeImpulses );
		o.set( K( "splitIslands" ), p.splitIslands );
		o.set( K( "transforms" ), p.transforms );
		o.set( K( "sensorHits" ), p.sensorHits );
		o.set( K( "jointEvents" ), p.jointEvents );
		o.set( K( "hitEvents" ), p.hitEvents );
		o.set( K( "refit" ), p.refit );
		o.set( K( "bullets" ), p.bullets );
		o.set( K( "sleepIslands" ), p.sleepIslands );
		o.set( K( "sensors" ), p.sensors );
		return o;
	}
};

// ---------------------------------------------------------------------------
// Step timers
// ---------------------------------------------------------------------------

// Box3D reads the clock around every stage of a step and every time-of-impact sweep, for its profile and a stall log.
// In WebAssembly each read crosses into JavaScript: 1 241 reads a step on 360 bodies, 5 % to 15 % of the step. The
// single-threaded flavours link with --wrap=clock_gettime (scripts/build.sh), so the clock reads zero until asked for.
// Without the timers World.getProfile is all zeros and the stall log stays silent; results do not depend on them.
#ifdef __EMSCRIPTEN_PTHREADS__
static void enableStepTimers( bool ) {}
#else
static bool s_stepTimers = false;

extern "C" int __real_clock_gettime( clockid_t id, struct timespec* ts );
extern "C" int __wrap_clock_gettime( clockid_t id, struct timespec* ts )
{
	if ( s_stepTimers )
	{
		return __real_clock_gettime( id, ts );
	}
	ts->tv_sec = 0;
	ts->tv_nsec = 0;
	return 0;
}

static void enableStepTimers( bool on )
{
	s_stepTimers = on;
}
#endif

// ---------------------------------------------------------------------------
// Bindings
// ---------------------------------------------------------------------------

EMSCRIPTEN_BINDINGS( box3d )
{
	using namespace emscripten;

	function( "enableStepTimers", &enableStepTimers );

#ifdef __EMSCRIPTEN_PTHREADS__
	constant( "threaded", true );
#else
	constant( "threaded", false );
#endif
	constant( "maxWorkers", (int)B3_MAX_WORKERS );
	function( "solvePlanes", &solvePlanes );
	function( "clipVector", &clipVector );
	function( "flatIn", &flatIn );
	function( "flatOut", &flatOut );

	class_<World>( "World" )
		.constructor<>()
		.constructor<val>()
		.function( "isValid", &World::isValid )
		.function( "destroy", &World::destroy )
		.function( "save", &World::save )
		.function( "restore", &World::restore )
		.function( "getBody", &World::getBody )
		.function( "step", &World::step )
		.function( "getGravity", &World::getGravity )
		.function( "setGravity", &World::setGravity )
		.function( "enableSleeping", &World::enableSleeping )
		.function( "enableContinuous", &World::enableContinuous )
		.function( "getAwakeBodyCount", &World::getAwakeBodyCount )
		.function( "getWorkerCount", &World::getWorkerCount )
		.function( "createBody", &World::createBody )
		.function( "createDistanceJoint", &World::createDistanceJoint )
		.function( "createRevoluteJoint", &World::createRevoluteJoint )
		.function( "createSphericalJoint", &World::createSphericalJoint )
		.function( "createPrismaticJoint", &World::createPrismaticJoint )
		.function( "createWeldJoint", &World::createWeldJoint )
		.function( "createMotorJoint", &World::createMotorJoint )
		.function( "createWheelJoint", &World::createWheelJoint )
		.function( "createParallelJoint", &World::createParallelJoint )
		.function( "createFilterJoint", &World::createFilterJoint )
		.function( "castRayClosest", &World::castRayClosest )
		.function( "castRay", &World::castRay )
		.function( "castShape", &World::castShape )
		.function( "overlapShape", &World::overlapShape )
		.function( "closestPoint", &World::closestPoint )
		.function( "queryBatch", &World::queryBatch )
		.function( "readMoved", &World::readMoved )
		.function( "readEvents", &World::readEvents )
		.function( "writeBodies", &World::writeBodies )
		.function( "moverBatch", &World::moverBatch )
		.function( "castMover", &World::castMover )
		.function( "collideMover", &World::collideMover )
		.function( "explode", &World::explode )
		.function( "getBodyEvents", &World::getBodyEvents )
		.function( "getBodyMoveEvents", &World::getBodyMoveEvents )
		.function( "getContactEvents", &World::getContactEvents )
		.function( "getSensorEvents", &World::getSensorEvents )
		.function( "getJointEvents", &World::getJointEvents )
		.function( "getProfile", &World::getProfile );

	class_<Body>( "Body" )
		.function( "isValid", &Body::isValid )
		.function( "destroy", &Body::destroy )
		.function( "getType", &Guarded<&Body::getType>::call )
		.function( "setType", &Guarded<&Body::setType>::call )
		.function( "getName", &Guarded<&Body::getName>::call )
		.function( "setName", &Guarded<&Body::setName>::call )
		.function( "getUserData", &Guarded<&Body::getUserData>::call )
		.function( "setUserData", &Guarded<&Body::setUserData>::call )
		.function( "getIndex", &Body::getIndex )
		.function( "getGeneration", &Body::getGeneration )
		.function( "getPosition", &Guarded<&Body::getPosition>::call )
		.function( "getRotation", &Guarded<&Body::getRotation>::call )
		.function( "getTransform", &Guarded<&Body::getTransform>::call )
		.function( "setTransform", &Guarded<&Body::setTransform>::call )
		.function( "setTargetTransform", &Guarded<&Body::setTargetTransform>::call )
		.function( "getLinearVelocity", &Guarded<&Body::getLinearVelocity>::call )
		.function( "setLinearVelocity", &Guarded<&Body::setLinearVelocity>::call )
		.function( "getAngularVelocity", &Guarded<&Body::getAngularVelocity>::call )
		.function( "setAngularVelocity", &Guarded<&Body::setAngularVelocity>::call )
		.function( "applyForce", &Guarded<&Body::applyForce>::call )
		.function( "applyForceToCenter", &Guarded<&Body::applyForceToCenter>::call )
		.function( "applyTorque", &Guarded<&Body::applyTorque>::call )
		.function( "applyLinearImpulse", &Guarded<&Body::applyLinearImpulse>::call )
		.function( "applyLinearImpulseToCenter", &Guarded<&Body::applyLinearImpulseToCenter>::call )
		.function( "applyAngularImpulse", &Guarded<&Body::applyAngularImpulse>::call )
		.function( "getMass", &Guarded<&Body::getMass>::call )
		.function( "applyMassFromShapes", &Guarded<&Body::applyMassFromShapes>::call )
		.function( "getLocalCenterOfMass", &Guarded<&Body::getLocalCenterOfMass>::call )
		.function( "getWorldCenterOfMass", &Guarded<&Body::getWorldCenterOfMass>::call )
		.function( "getLocalPoint", &Guarded<&Body::getLocalPoint>::call )
		.function( "getWorldPoint", &Guarded<&Body::getWorldPoint>::call )
		.function( "getLinearDamping", &Guarded<&Body::getLinearDamping>::call )
		.function( "setLinearDamping", &Guarded<&Body::setLinearDamping>::call )
		.function( "getAngularDamping", &Guarded<&Body::getAngularDamping>::call )
		.function( "setAngularDamping", &Guarded<&Body::setAngularDamping>::call )
		.function( "getGravityScale", &Guarded<&Body::getGravityScale>::call )
		.function( "setGravityScale", &Guarded<&Body::setGravityScale>::call )
		.function( "isAwake", &Guarded<&Body::isAwake>::call )
		.function( "setAwake", &Guarded<&Body::setAwake>::call )
		.function( "enableSleep", &Guarded<&Body::enableSleep>::call )
		.function( "isEnabled", &Guarded<&Body::isEnabled>::call )
		.function( "setEnabled", &Guarded<&Body::setEnabled>::call )
		.function( "isBullet", &Guarded<&Body::isBullet>::call )
		.function( "setBullet", &Guarded<&Body::setBullet>::call )
		.function( "setMotionLocks", &Guarded<&Body::setMotionLocks>::call )
		.function( "getMotionLocks", &Guarded<&Body::getMotionLocks>::call )
		.function( "getShapeCount", &Guarded<&Body::getShapeCount>::call )
		.function( "computeAABB", &Guarded<&Body::computeAABB>::call )
		.function( "createHeightField", &Guarded<&Body::createHeightField>::call )
		.function( "createSphere", &Guarded<&Body::createSphere>::call )
		.function( "createCapsule", &Guarded<&Body::createCapsule>::call )
		.function( "createBox", &Guarded<&Body::createBox>::call )
		.function( "createHull", &Guarded<&Body::createHull>::call )
		.function( "createMesh", &Guarded<&Body::createMesh>::call );

	class_<Shape>( "Shape" )
		.function( "isValid", &Shape::isValid )
		.function( "destroy", &Shape::destroy )
		.function( "getType", &Guarded<&Shape::getType>::call )
		.function( "getUserData", &Guarded<&Shape::getUserData>::call )
		.function( "setUserData", &Guarded<&Shape::setUserData>::call )
		.function( "getFriction", &Guarded<&Shape::getFriction>::call )
		.function( "setFriction", &Guarded<&Shape::setFriction>::call )
		.function( "getRestitution", &Guarded<&Shape::getRestitution>::call )
		.function( "setRestitution", &Guarded<&Shape::setRestitution>::call )
		.function( "getDensity", &Guarded<&Shape::getDensity>::call )
		.function( "setDensity", &Guarded<&Shape::setDensity>::call )
		.function( "isSensor", &Guarded<&Shape::isSensor>::call )
		.function( "enableSensorEvents", &Guarded<&Shape::enableSensorEvents>::call )
		.function( "enableContactEvents", &Guarded<&Shape::enableContactEvents>::call )
		.function( "getContactData", &Guarded<&Shape::getContactData>::call )
		.function( "enableHitEvents", &Guarded<&Shape::enableHitEvents>::call )
		.function( "getFilter", &Guarded<&Shape::getFilter>::call )
		.function( "setFilter", &Guarded<&Shape::setFilter>::call )
		.function( "getAABB", &Guarded<&Shape::getAABB>::call )
		.function( "getClosestPoint", &Guarded<&Shape::getClosestPoint>::call )
		.function( "rayCast", &Guarded<&Shape::rayCast>::call );

	class_<Joint>( "Joint" )
		.function( "isValid", &Joint::isValid )
		.function( "destroy", &Joint::destroy )
		.function( "getType", &Guarded<&Joint::getType>::call )
		.function( "wakeBodies", &Guarded<&Joint::wakeBodies>::call )
		.function( "getCollideConnected", &Guarded<&Joint::getCollideConnected>::call )
		.function( "setCollideConnected", &Guarded<&Joint::setCollideConnected>::call )
		.function( "getLocalFrameA", &Guarded<&Joint::getLocalFrameA>::call )
		.function( "getLocalFrameB", &Guarded<&Joint::getLocalFrameB>::call )
		.function( "getConstraintForce", &Guarded<&Joint::getConstraintForce>::call )
		.function( "getConstraintTorque", &Guarded<&Joint::getConstraintTorque>::call )
		.function( "setForceThreshold", &Guarded<&Joint::setForceThreshold>::call )
		.function( "setTorqueThreshold", &Guarded<&Joint::setTorqueThreshold>::call )
		.function( "getIndex", &Joint::getIndex );

	class_<DistanceJoint, base<Joint>>( "DistanceJoint" )
		.function( "setLength", &Guarded<&DistanceJoint::setLength>::call )
		.function( "getLength", &Guarded<&DistanceJoint::getLength>::call )
		.function( "getCurrentLength", &Guarded<&DistanceJoint::getCurrentLength>::call )
		.function( "enableSpring", &Guarded<&DistanceJoint::enableSpring>::call )
		.function( "setSpringHertz", &Guarded<&DistanceJoint::setSpringHertz>::call )
		.function( "setSpringDampingRatio", &Guarded<&DistanceJoint::setSpringDampingRatio>::call )
		.function( "enableLimit", &Guarded<&DistanceJoint::enableLimit>::call )
		.function( "setLengthRange", &Guarded<&DistanceJoint::setLengthRange>::call )
		.function( "enableMotor", &Guarded<&DistanceJoint::enableMotor>::call )
		.function( "setMotorSpeed", &Guarded<&DistanceJoint::setMotorSpeed>::call )
		.function( "setMaxMotorForce", &Guarded<&DistanceJoint::setMaxMotorForce>::call );

	class_<RevoluteJoint, base<Joint>>( "RevoluteJoint" )
		.function( "getAngle", &Guarded<&RevoluteJoint::getAngle>::call )
		.function( "enableSpring", &Guarded<&RevoluteJoint::enableSpring>::call )
		.function( "setSpringHertz", &Guarded<&RevoluteJoint::setSpringHertz>::call )
		.function( "setSpringDampingRatio", &Guarded<&RevoluteJoint::setSpringDampingRatio>::call )
		.function( "setTargetAngle", &Guarded<&RevoluteJoint::setTargetAngle>::call )
		.function( "enableLimit", &Guarded<&RevoluteJoint::enableLimit>::call )
		.function( "setLimits", &Guarded<&RevoluteJoint::setLimits>::call )
		.function( "enableMotor", &Guarded<&RevoluteJoint::enableMotor>::call )
		.function( "setMotorSpeed", &Guarded<&RevoluteJoint::setMotorSpeed>::call )
		.function( "setMaxMotorTorque", &Guarded<&RevoluteJoint::setMaxMotorTorque>::call )
		.function( "getMotorTorque", &Guarded<&RevoluteJoint::getMotorTorque>::call );

	class_<SphericalJoint, base<Joint>>( "SphericalJoint" )
		.function( "enableConeLimit", &Guarded<&SphericalJoint::enableConeLimit>::call )
		.function( "setConeLimit", &Guarded<&SphericalJoint::setConeLimit>::call )
		.function( "getConeAngle", &Guarded<&SphericalJoint::getConeAngle>::call )
		.function( "enableTwistLimit", &Guarded<&SphericalJoint::enableTwistLimit>::call )
		.function( "setTwistLimits", &Guarded<&SphericalJoint::setTwistLimits>::call )
		.function( "getTwistAngle", &Guarded<&SphericalJoint::getTwistAngle>::call )
		.function( "enableSpring", &Guarded<&SphericalJoint::enableSpring>::call )
		.function( "setSpringHertz", &Guarded<&SphericalJoint::setSpringHertz>::call )
		.function( "setSpringDampingRatio", &Guarded<&SphericalJoint::setSpringDampingRatio>::call )
		.function( "setTargetRotation", &Guarded<&SphericalJoint::setTargetRotation>::call )
		.function( "enableMotor", &Guarded<&SphericalJoint::enableMotor>::call )
		.function( "setMotorVelocity", &Guarded<&SphericalJoint::setMotorVelocity>::call )
		.function( "setMaxMotorTorque", &Guarded<&SphericalJoint::setMaxMotorTorque>::call );

	class_<PrismaticJoint, base<Joint>>( "PrismaticJoint" )
		.function( "getTranslation", &Guarded<&PrismaticJoint::getTranslation>::call )
		.function( "getSpeed", &Guarded<&PrismaticJoint::getSpeed>::call )
		.function( "enableSpring", &Guarded<&PrismaticJoint::enableSpring>::call )
		.function( "setSpringHertz", &Guarded<&PrismaticJoint::setSpringHertz>::call )
		.function( "setSpringDampingRatio", &Guarded<&PrismaticJoint::setSpringDampingRatio>::call )
		.function( "setTargetTranslation", &Guarded<&PrismaticJoint::setTargetTranslation>::call )
		.function( "enableLimit", &Guarded<&PrismaticJoint::enableLimit>::call )
		.function( "setLimits", &Guarded<&PrismaticJoint::setLimits>::call )
		.function( "enableMotor", &Guarded<&PrismaticJoint::enableMotor>::call )
		.function( "setMotorSpeed", &Guarded<&PrismaticJoint::setMotorSpeed>::call )
		.function( "setMaxMotorForce", &Guarded<&PrismaticJoint::setMaxMotorForce>::call );

	class_<WeldJoint, base<Joint>>( "WeldJoint" )
		.function( "setLinearHertz", &Guarded<&WeldJoint::setLinearHertz>::call )
		.function( "setLinearDampingRatio", &Guarded<&WeldJoint::setLinearDampingRatio>::call )
		.function( "setAngularHertz", &Guarded<&WeldJoint::setAngularHertz>::call )
		.function( "setAngularDampingRatio", &Guarded<&WeldJoint::setAngularDampingRatio>::call );

	class_<MotorJoint, base<Joint>>( "MotorJoint" )
		.function( "setLinearVelocity", &Guarded<&MotorJoint::setLinearVelocity>::call )
		.function( "setAngularVelocity", &Guarded<&MotorJoint::setAngularVelocity>::call )
		.function( "setMaxVelocityForce", &Guarded<&MotorJoint::setMaxVelocityForce>::call )
		.function( "setMaxVelocityTorque", &Guarded<&MotorJoint::setMaxVelocityTorque>::call )
		.function( "setLinearHertz", &Guarded<&MotorJoint::setLinearHertz>::call )
		.function( "setLinearDampingRatio", &Guarded<&MotorJoint::setLinearDampingRatio>::call )
		.function( "setAngularHertz", &Guarded<&MotorJoint::setAngularHertz>::call )
		.function( "setAngularDampingRatio", &Guarded<&MotorJoint::setAngularDampingRatio>::call )
		.function( "setMaxSpringForce", &Guarded<&MotorJoint::setMaxSpringForce>::call )
		.function( "setMaxSpringTorque", &Guarded<&MotorJoint::setMaxSpringTorque>::call );

	class_<WheelJoint, base<Joint>>( "WheelJoint" )
		.function( "enableSuspension", &Guarded<&WheelJoint::enableSuspension>::call )
		.function( "setSuspensionHertz", &Guarded<&WheelJoint::setSuspensionHertz>::call )
		.function( "setSuspensionDampingRatio", &Guarded<&WheelJoint::setSuspensionDampingRatio>::call )
		.function( "enableSuspensionLimit", &Guarded<&WheelJoint::enableSuspensionLimit>::call )
		.function( "setSuspensionLimits", &Guarded<&WheelJoint::setSuspensionLimits>::call )
		.function( "enableSpinMotor", &Guarded<&WheelJoint::enableSpinMotor>::call )
		.function( "setSpinMotorSpeed", &Guarded<&WheelJoint::setSpinMotorSpeed>::call )
		.function( "setMaxSpinTorque", &Guarded<&WheelJoint::setMaxSpinTorque>::call )
		.function( "getSpinSpeed", &Guarded<&WheelJoint::getSpinSpeed>::call )
		.function( "enableSteering", &Guarded<&WheelJoint::enableSteering>::call )
		.function( "setSteeringHertz", &Guarded<&WheelJoint::setSteeringHertz>::call )
		.function( "setSteeringDampingRatio", &Guarded<&WheelJoint::setSteeringDampingRatio>::call )
		.function( "setMaxSteeringTorque", &Guarded<&WheelJoint::setMaxSteeringTorque>::call )
		.function( "enableSteeringLimit", &Guarded<&WheelJoint::enableSteeringLimit>::call )
		.function( "setSteeringLimits", &Guarded<&WheelJoint::setSteeringLimits>::call )
		.function( "setTargetSteeringAngle", &Guarded<&WheelJoint::setTargetSteeringAngle>::call )
		.function( "getSteeringAngle", &Guarded<&WheelJoint::getSteeringAngle>::call );

	class_<ParallelJoint, base<Joint>>( "ParallelJoint" )
		.function( "setSpringHertz", &Guarded<&ParallelJoint::setSpringHertz>::call )
		.function( "setSpringDampingRatio", &Guarded<&ParallelJoint::setSpringDampingRatio>::call )
		.function( "setMaxTorque", &Guarded<&ParallelJoint::setMaxTorque>::call );

	class_<FilterJoint, base<Joint>>( "FilterJoint" );
}
