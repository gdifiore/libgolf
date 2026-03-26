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

## Running Locally

Both files must be served together over HTTP (browsers block wasm from `file://`):

```sh
cd build-wasm
python3 -m http.server 8080
```

## JavaScript API

The module exports a single factory function. Import it as an ES6 module:

```js
import createLibGolf from './libgolf_wasm.js';

const libgolf = await createLibGolf();
```

### `runTrajectory()`

Runs a full simulation and returns the trajectory as a flat position array.

```js
const result = libgolf.runTrajectory(
    ballSpeedMph,   // float — e.g. 160.0
    launchAngleDeg, // float — e.g. 12.0
    directionDeg,   // float — 0 = straight, + = right, - = left
    backspinRpm,    // float — e.g. 3000.0
    sidespinRpm,    // float — + = hook spin, - = slice spin
    windSpeedMph,   // float — e.g. 10.0
    windDirDeg      // float — 0 = crosswind left, 90 = headwind, 180 = tailwind
);

// result is an Emscripten VectorFloat proxy
const size = result.size(); // total number of floats (points × 3)
for (let i = 0; i < size; i += 3) {
    const x = result.get(i);    // lateral position in yards (right = +)
    const y = result.get(i + 1); // downrange position in yards (forward = +)
    const z = result.get(i + 2); // height in yards (up = +)
}

result.delete(); // must be called to free wasm memory
```

The returned object is heap-allocated in wasm memory. Forgetting `.delete()` leaks memory across calls.

## Coordinate System

Positions are in **yards**, using the same axes as the rest of the library:

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

## Atmospheric Conditions

`runTrajectory` uses standard sea-level atmosphere (70°F, 50% humidity, 29.92 inHg). Wind is the only atmospheric parameter exposed in the wasm API. For full atmospheric control, use the C++ library directly.

## Release Artifacts

Each release includes a pre-built `libgolf-wasm-<version>.zip` containing `libgolf_wasm.js` and `libgolf_wasm.wasm`. Download and serve them alongside your HTML — no build step required for consumers.
