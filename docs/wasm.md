# WebAssembly Build

libgolf can be compiled to WebAssembly using [Emscripten](https://emscripten.org), producing a JavaScript module that runs in any modern browser.

## Prerequisites

Install the Emscripten SDK and activate it:

```sh
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh  # add to your shell profile to persist
```

`EMSDK` must be set in your environment — the CMake preset reads the toolchain path from it.

## Building

```sh
cmake --preset wasm
cmake --build --preset wasm
```

Output lands in `build-wasm/`:
- `libgolf_wasm.js` — ES6 module loader
- `libgolf_wasm.wasm` — compiled library

## Running the Visualizer Locally

The visualizer source lives in [`examples/web/index.html`](../examples/web/index.html). It loads `./libgolf_wasm.js` from the same directory, so copy the build artifacts next to it before serving:

```sh
cp build-wasm/libgolf_wasm.{js,wasm} examples/web/
cd examples/web
python3 -m http.server 8080
```

Browsers block wasm from `file://`, so a local server is required.

The same files are deployed to GitHub Pages by `.github/workflows/pages.yml` whenever the visualizer or library sources change.

## JavaScript API

The module exports a single factory function. Import it as an ES6 module:

```js
import createLibGolf from './libgolf_wasm.js';

const libgolf = await createLibGolf();
```

### `runShot(launch, atmos, ground)`

Runs a full simulation (aerial → bounce → roll) and returns a `ShotResult`. All three input objects are passed as plain JS object literals; field names match the C++ structs.

```js
const result = libgolf.runShot(
    {
        ballSpeedMph:   160.0,
        launchAngleDeg: 12.0,
        directionDeg:   0.0,    // 0 = straight, + = right, − = left
        backspinRpm:    3000.0,
        sidespinRpm:    0.0,    // + = hook, − = slice
        startX: 0, startY: 0, startZ: 0,
    },
    {
        temp:        70.0,      // °F
        elevation:   0.0,       // ft
        vWind:       10.0,      // mph
        phiWind:     90.0,      // 0 = cross-L, 90 = head, 180 = tail, 270 = cross-R
        hWind:       0.0,
        relHumidity: 50.0,      // %
        pressure:    29.92,     // inHg
    },
    {
        height:          0.0,
        restitution:     0.45,
        frictionStatic:  0.50,
        frictionDynamic: 0.20,
        firmness:        0.85,
        spinRetention:   0.75,
        criticalAngle:   15.0 * Math.PI / 180.0,
    }
);
```

`ShotResult` fields:

| Field | Type | Description |
|------|------|-------------|
| `trajectory`   | `VectorFloat` | Flat `[x, y, z, x, y, z, ...]` array of positions in **yards** |
| `carryIndex`   | `number`      | Index (in points, not floats) of first ground contact |
| `carryYards`   | `number`      | Downrange distance at first ground contact |
| `totalYards`   | `number`      | Downrange distance at rest (carry + roll) |
| `apexYards`    | `number`      | Peak height above ground |
| `offlineYards` | `number`      | Lateral position at rest (right = +) |
| `timeOfFlight` | `number`      | Total simulation time, seconds |
| `bearingDeg`   | `number`      | Bearing from launch to rest, degrees |

Walk the trajectory:

```js
const traj = result.trajectory;
const n = traj.size();
for (let i = 0; i < n; i += 3) {
    const x = traj.get(i);     // lateral, right = +
    const y = traj.get(i + 1); // downrange, forward = +
    const z = traj.get(i + 2); // height, up = +
}
traj.delete(); // free wasm heap memory
```

The vector is heap-allocated in wasm. Forgetting `.delete()` leaks memory across calls — the surrounding `result` object is plain JS and does not need to be released.

## Coordinate System

Trajectory positions are in **yards**, using the same axes as the rest of the library:

| Axis | Direction |
|------|-----------|
| x    | Lateral — positive = right of target line |
| y    | Downrange — positive = toward target |
| z    | Vertical — positive = up |

When mapping to a Y-up 3D engine (e.g. Three.js):
```js
threeX = x;   // lateral unchanged
threeY = z;   // height → up axis
threeZ = -y;  // forward → negative Z
```

## Release Artifacts

Each release includes a pre-built `libgolf-wasm-<version>.zip` containing `libgolf_wasm.js` and `libgolf_wasm.wasm`. Download and serve them alongside your HTML — no build step required for consumers.
