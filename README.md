# box3d-efa

[Box3D](https://github.com/erincatto/box3d) compiled to WebAssembly for the EFA project, the binding its physics adapter
uses. **It is built for EFA's needs:** it is public, but it does not promise general use, a stable API or support, and it
changes when EFA needs it to. It started from [monteslu/box3d-wasm](https://github.com/monteslu/box3d-wasm) (MIT, see
`LICENSE`). Box3D itself is taken from its own repository at a pinned SHA, and its C code is not changed.

## What it adds

The binding adds to the upstream embind glue (`csrc/glue.cpp`):

| JS | C |
|---|---|
| `World.castRay(origin, translation, filter)` | `b3World_CastRay` (closest hit, names the body by index) |
| `World.castShape(origin, { points, radius }, translation, filter)` | `b3World_CastShape` (closest hit) |
| `World.overlapShape(origin, { points, radius }, filter)` | `b3World_OverlapShape` (every shape) |
| `World.closestPoint(target, maxDistance, filter)`, `Shape.getClosestPoint(target)` | the broadphase boxes within `maxDistance` (`b3World_OverlapAABB`), then `b3Shape_GetClosestPoint` per convex shape (meshes, height fields and compounds are left out: Box3D has no closest point on them) |
| `World.castMover`, `World.collideMover`, `solvePlanes`, `clipVector` | `b3World_CastMover`, `b3World_CollideMover`, `b3SolvePlanes`, `b3ClipVector` |
| `Body.createMesh({ vertices, indices })` | `b3CreateMesh`, `b3CreateMeshShape` |
| `Body.createHeightField({ heights, countX, countZ, scale, ... })` | `b3CreateHeightField`, `b3CreateHeightFieldShape` |
| `World.getBodyMoveEvents()` | `b3World_GetBodyEvents` as typed arrays (`bodyIndex`, `userData`, `transform`, `fellAsleep`) |
| `World.save()`, `World.restore(bytes)`, `World.getBody(index, generation)`, `Body.getIndex()`, `Body.getGeneration()` | `b3World_StartRecording`, `b3World_StopRecording`, `b3RecPlayer_Create` |
| `World.getJointEvents()`, `Joint.setForceThreshold`, `Joint.setTorqueThreshold`, `Joint.getIndex()` | `b3World_GetJointEvents`, `b3Joint_SetForceThreshold`, `b3Joint_SetTorqueThreshold` |
| `Shape.getContactData()` | `b3Shape_GetContactCapacity`, `b3Shape_GetContactData` |
| `flatIn(words)`, `flatOut()`, `World.queryBatch(count)`, `World.readMoved()`, `World.readEvents()`, `World.writeBodies(count)`, `World.moverBatch(count)` | the flat calls: records of 32-bit words in the module's memory, no JS objects (queries; moved bodies with their velocities; contact and sensor events; velocity, transform, target transform and impulse writes; a mover's move of up to five collide, solve and cast rounds, a mover cast, the ground under a mover). Layouts in `csrc/glue.cpp`, "Flat calls" |

The glue reads an argument object's vector or quaternion, and builds a result object, in one call into JS (`EM_JS`),
and makes each property key's JS string once: an object call crosses between JS and WebAssembly a few times instead of
tens. The API is unchanged; on Node the object calls take 0.12x to 0.67x of their former time, the flat calls less.

A second `destroy()` of a body, shape, joint or world does nothing, and any other method on a handle whose object
or world was destroyed throws (before, a stale id read or wrote whatever held its slot). `test/reference.test.mjs` runs seeded scenarios of object calls on this build and on a reference build of
the files of a previous release on the same Box3D SHA (another Box3D gives other floats) and compares every result: `B3_REFERENCE=<folder with the release's files> npm test` (skipped without it).

World queries leave sensor shapes out unless the filter has `includeSensors: true`. 
The single-threaded flavours (`standard`, `scalar`) are `-Os` with LTO and read no clock inside a step: under Emscripten
each read crosses into JavaScript, and Box3D's timing would otherwise cost much of a step on a phone. So `World.getProfile`
is all zeros until `enableStepTimers(true)` is called, and Box3D's stall log is silent. `deluxe` keeps its timers and `-O3`.

[Box3D](https://github.com/erincatto/box3d) compiled to WebAssembly, with SIMD and optional wasm threads. Works in browsers and Node.js from a single package.

See it in action: [live three.js demo](https://box3d.netlify.app/) ([source](https://github.com/monteslu/threejs-box3d-demo)) with ragdolls, dominoes, a drivable buggy, and more.

Box3D is a 3D rigid body physics engine written by Erin Catto, the author of Box2D. All engine design and implementation credit belongs to him. This package only compiles his library to wasm and adds a JavaScript binding layer. Box3D is MIT licensed by Erin Catto; the full upstream license ships in this package as LICENSE.box3d.txt.

## Install

Not on npm. Each `v*` tag is a [GitHub release](https://github.com/VirtualMaestro/box3d-efa/releases) with the built files and
their `SHA256SUMS.txt`, made by CI.

## Quick start

```js
import Box3D from 'box3d-efa';

const b3 = await Box3D();

const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });

const ground = world.createBody({ type: 'static', position: { x: 0, y: -0.5, z: 0 } });
ground.createBox({ halfExtents: { x: 20, y: 0.5, z: 20 } });

const body = world.createBody({ type: 'dynamic', position: { x: 0, y: 5, z: 0 } });
body.createBox({ halfExtents: { x: 0.5, y: 0.5, z: 0.5 }, density: 1, friction: 0.5 });

for (let i = 0; i < 120; i++) {
  world.step(1 / 60, 4);
}

console.log(body.getPosition()); // { x: ~0, y: ~0.5, z: ~0 }
```

The same code runs in Node.js and in the browser. The default import checks wasm SIMD and thread support and loads the build that fits. The wasm file is loaded relative to the module, so bundlers that understand `new URL(..., import.meta.url)` (Vite, webpack 5, Rollup) pick it up automatically.

## Flavours

| import | threads | picked by the default import when |
| --- | --- | --- |
| `box3d-efa/scalar` | no | the host has no wasm SIMD |
| `box3d-efa/deluxe` | yes | SharedArrayBuffer is usable (Node.js, or a cross-origin isolated page) |
| `box3d-efa/standard` | no | everything else |

`box3d-efa` (the default import) runs the detection above and returns whichever module fits. Import a specific flavour directly when you want to skip detection:

```js
import Box3D from 'box3d-efa/deluxe';

const b3 = await Box3D();
const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 }, workerCount: 4 });
```

`workerCount` enables Box3D's internal multithreaded solver. It is clamped to `[1, b3.maxWorkers]`. The single threaded builds ignore it. Check `b3.threaded` at runtime to see which build you got.

### Serving requirements for threads

Wasm threads use SharedArrayBuffer. Browsers require the page to be cross-origin isolated, so serve your app with:

```
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```

Node.js needs no special setup; worker threads are used automatically.

## API overview

Vectors are plain objects `{ x, y, z }` and quaternions are `{ x, y, z, w }`, so values pass directly to and from libraries like three.js.

### World

```js
const world = new b3.World({
  gravity: { x: 0, y: -10, z: 0 },
  enableSleep: true,
  enableContinuous: true,
  workerCount: 4, // deluxe build only
});

world.step(1 / 60, 4);          // timeStep, subStepCount
world.setGravity({ x: 0, y: -3.7, z: 0 });
world.getAwakeBodyCount();
world.castRayClosest(origin, translation, filter);
world.explode({ position, radius: 3, falloff: 2, impulsePerArea: 10 });
world.getBodyEvents();          // [{ userData, position, rotation, fellAsleep }]
world.getContactEvents();       // { begin: [...], end: [...], hit: [...] }
world.getSensorEvents();        // { begin: [...], end: [...] }
world.destroy();
```

### Bodies

```js
const body = world.createBody({
  type: 'dynamic',              // 'static' | 'kinematic' | 'dynamic'
  position: { x: 0, y: 5, z: 0 },
  rotation: { x: 0, y: 0, z: 0, w: 1 },
  linearVelocity: { x: 0, y: 0, z: 0 },
  angularDamping: 0.05,
  motionLocks: { angularX: true, angularZ: true },
});

body.getPosition(); body.getRotation(); body.getTransform();
body.setLinearVelocity(v); body.applyLinearImpulseToCenter(v, true);
body.applyForce(force, worldPoint, true); body.applyTorque(t, true);
body.getMass(); body.isAwake(); body.setAwake(true);
body.destroy();
```

### Shapes

Each shape creator takes one options object with the geometry plus material fields (`density`, `friction`, `restitution`, `isSensor`, `filter`, event flags):

```js
body.createBox({ halfExtents: { x: 1, y: 0.5, z: 2 }, friction: 0.7 });
body.createSphere({ radius: 0.5, restitution: 0.8 });
body.createCapsule({ height: 1.2, radius: 0.3 });
body.createHull({ points: [{ x, y, z }, ...] });
```

### Joints

Distance, revolute, spherical, prismatic, weld, motor, wheel, parallel, and filter joints are bound:

```js
const hinge = world.createRevoluteJoint(bodyA, bodyB, {
  localFrameA: { position: { x: 0, y: 1, z: 0 } },
  enableMotor: true,
  motorSpeed: 5,
  maxMotorTorque: 100,
});
hinge.getAngle();
```

### Events and userData tags

Every body and shape gets an auto-assigned numeric `userData` tag (you can overwrite it with your own number). Event arrays reference these tags, so you can map physics events back to your scene objects with a plain `Map`.

### Memory notes

Wrapper objects returned by embind (`World`, `Body`, `Shape`, joints) are tiny handles. Call `.delete()` when you no longer need the JS handle, and `.destroy()` to remove the underlying object from the simulation. Destroying a world frees every body, shape, and joint inside it.

## The isomorphic surface (`box3d-efa/iso`)

For projects where the same game source also compiles to a NATIVE binary
(e.g. [scriptc-game](https://github.com/monteslu/scriptc-game)), the
`/iso` entry serves the same engine through a shared binding frontend:

```js
import Box3D from 'box3d-efa/iso';

const b3 = await Box3D();
const world = new b3.World({ gravity: { x: 0, y: -10, z: 0 } });
```

`src/frontend.ts` is the entire API personality, written against a flat
scalar contract (`src/backend.d.ts`). Here it drives the wasm build; a
native host vendors the identical file and implements the same contract
over FFI against Box3D built as a static library at the same pinned SHA.
One binding source, two worlds.

The iso surface currently covers worlds (step, gravity, explode, worker
threads), rigid bodies (create/read, velocities, impulses, teleports,
gravity scale, motion locks) and box/sphere shapes; it grows toward parity
with the embind API above, which remains the default export.

## Building from source

Requires [emsdk](https://emscripten.org/docs/getting_started/downloads.html) (6.0.13, the version CI pins), CMake, and Node 22+.

```bash
npm ci
npm run fetch-deps   # clones Box3D at the SHA pinned in scripts/versions.json
npm run build        # builds the standard, scalar and deluxe flavours into dist/
npm test
```

### Updating Box3D

Box3D is under active development and is not forked here. To move to a newer version: set its SHA in
`scripts/versions.json`, run `npm run fetch-deps`, build, and
fix the glue where its API changed. A new version can change simulation results: EFA's cross-host hashes and timing rows
are rerun before its adapter takes the new build.

## License

MIT for the wrapper and build scripts, see LICENSE.

Box3D itself is Copyright (c) Erin Catto and MIT licensed, see LICENSE.box3d.txt and the upstream repository at https://github.com/erincatto/box3d. If you use this package, the physics engine you are running is his work.
