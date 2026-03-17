# Terrain System

## Overview

The terrain system provides a flexible interface for simulating golf ball interactions with varying ground conditions. The `TerrainInterface` abstraction allows for custom terrain implementations including flat surfaces, slopes, heightmaps, and procedurally generated landscapes.

## Choosing Between TerrainInterface and GroundProvider

The library provides two interfaces for customizing ground behavior. Choose based on whether you need elevation changes:

### TerrainInterface

Use when you need 3D terrain:
- Elevation changes (hills, slopes, elevated greens)
- Varying surface normals for realistic slope physics
- Direct control over height, normal vector, and surface properties at any position

### GroundProvider

Use for flat terrain with varying materials:
- Different surface properties at different positions (fairway, rough, green)
- Simpler implementation - only surface properties, no geometric calculations
- Backward compatibility with existing code

Note: `GroundProvider` is internally wrapped in a `TerrainProviderAdapter` that assumes flat terrain. The height comes from the surface properties, and the normal is always vertical. For sloped terrain, use `TerrainInterface`.

## Quick Start

For flat terrain (backward compatible with existing code):

```c++
#include <terrain_interface.hpp>

// (height, restitution, frictionStatic, frictionDynamic, firmness, spinRetention)
GroundSurface ground{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};

auto terrain = std::make_shared<FlatTerrain>(ground);
FlightSimulator sim(ball, atmos, ground, terrain);
```

When a terrain is provided, the flight simulator queries terrain properties at each position during the simulation.

## Terrain Interface

The `TerrainInterface` defines three required methods:

### getHeight()

Returns the terrain elevation at a given horizontal position:

```c++
float getHeight(float x, float y) const override
{
    // Return terrain height (z-coordinate) in feet at position (x, y)
    return heightmap.lookup(x, y);
}
```

### getNormal()

Returns the surface normal vector at a given position. The normal should be unit length and point upward (away from solid terrain, into the air):

```c++
Vector3D getNormal(float x, float y) const override
{
    // Compute gradient from heightmap
    Vector3D gradient = heightmap.computeGradient(x, y);

    // Normal is perpendicular to tangent plane
    Vector3D normal = {-gradient[0], -gradient[1], 1.0F};

    // Must return unit vector
    return math_utils::normalize(normal);
}
```

For flat surfaces, the normal is always `{0.0F, 0.0F, 1.0F}`. For sloped surfaces, the normal tilts accordingly while maintaining unit length.

**Important:** The normal vector should be unit length (magnitude = 1.0) for correct physics. Non-unit normals produce incorrect results but will not throw errors.

### getSurfaceProperties()

Returns material properties at a given position:

```c++
const GroundSurface& getSurfaceProperties(float x, float y) const override
{
    // Return properties based on terrain type at this location
    if (isInBunker(x, y))
        return bunkerSurface;
    else if (isOnGreen(x, y))
        return greenSurface;
    else
        return fairwaySurface;
}
```

## Implementing Custom Terrain

Example of a sloped terrain implementation:

```c++
class SlopedTerrain : public TerrainInterface
{
public:
    SlopedTerrain(float slopeAngleDegrees, const GroundSurface& surface)
        : surface_(surface)
    {
        float angleRad = slopeAngleDegrees * physics_constants::DEG_TO_RAD;
        slopeRise_ = std::tan(angleRad);

        // Precompute unit normal for this uniform slope
        normal_[0] = 0.0F;
        normal_[1] = std::sin(angleRad);
        normal_[2] = std::cos(angleRad);
    }

    float getHeight(float x, float y) const override
    {
        (void)x;  // Slope only varies in y direction
        return -y * slopeRise_;  // Descends as y increases
    }

    Vector3D getNormal(float x, float y) const override
    {
        (void)x;
        (void)y;
        return normal_;  // Constant for uniform slope
    }

    const GroundSurface& getSurfaceProperties(float x, float y) const override
    {
        (void)x;
        (void)y;
        return surface_;
    }

private:
    GroundSurface surface_;
    float slopeRise_;
    Vector3D normal_;
};
```

Usage:

```c++
GroundSurface fairway{0.0F, 0.4F, 0.5F, 0.15F, 0.8F, 0.75F};
auto terrain = std::make_shared<SlopedTerrain>(5.0F, fairway);  // 5-degree slope
FlightSimulator sim(ball, atmos, fairway, terrain);
```

## Ground Surface Properties

The `GroundSurface` struct defines physical characteristics:

### Restitution (Bounce)

```c++
ground.restitution = 0.4F;  // Range: 0.0 to 1.0
```

Coefficient of restitution controls bounce height. Higher values produce higher bounces:
- **0.8**: Hard cart path (80% energy retained)
- **0.4**: Fairway (default)
- **0.2**: Soft rough or sand

### Friction Coefficients

```c++
ground.frictionStatic = 0.5F;   // Affects bounce
ground.frictionDynamic = 0.2F;  // Affects roll
```

Friction reduces horizontal velocity:
- **Static friction** (0.0-1.0): Applied during bounce impact
- **Dynamic friction** (0.0-1.0): Applied during rolling phase

Higher friction values slow the ball more aggressively.

### Firmness

```c++
ground.firmness = 0.8F;  // Range: 0.0 to 1.0
```

Surface firmness modulates friction effectiveness:
- **1.0**: Very firm surface (minimal friction effect)
- **0.5**: Medium firmness
- **0.0**: Very soft surface (maximum friction effect)

Friction factor = `1.0 - frictionStatic * (1.0 - firmness)`

### Spin Retention

```c++
ground.spinRetention = 0.75F;  // Range: 0.0 to 1.0
```

Fraction of spin retained after bounce:
- **1.0**: All spin retained (unrealistic)
- **0.75**: Typical fairway (default)
- **0.5**: Soft surface with high spin loss

## Ground Physics

### Bounce Behavior

When the ball impacts the ground, velocity is decomposed into components normal and tangent to the surface:

- **Normal component**: Reversed and scaled by coefficient of restitution
- **Tangent component**: Reduced by friction based on firmness
- **Spin**: Reduced by spin retention factor

The physics correctly handle sloped surfaces, with bounce direction determined by the surface normal at the impact point.

### Roll Behavior

During the roll phase, two forces act on the ball:

1. **Gravity component along slope**: Accelerates ball downhill or decelerates uphill
2. **Rolling friction**: Opposes motion in all directions

On flat surfaces, rolling friction alone causes deceleration. On slopes, the ball will accelerate if the gravity component exceeds friction.

**Spin decay during roll:** Linear decay model where ground friction applies constant torque opposing spin. This differs from aerial phase which uses exponential decay due to aerodynamic damping.

### Phase Transitions

The simulation automatically transitions between phases:

- **Aerial → Bounce**: When ball reaches terrain height
- **Bounce → Roll**: When vertical velocity drops below threshold and ball is near ground
- **Roll → Complete**: When horizontal velocity drops below stopping threshold

## Validation and Error Handling

### Parameter Validation

The library does not validate physics parameters:

- Surface normals: should be unit vectors, not validated
- Surface properties: any values accepted (allows experimental physics)
- Caller's responsibility to ensure correct inputs

### Terrain Null Pointer

All flight phases require a valid terrain pointer:

```c++
if (!terrain)
{
    throw std::invalid_argument("Terrain interface must not be null");
}
```

Terrain validation occurs in constructor initialization, failing fast if invalid.

## Performance Considerations

Terrain queries occur multiple times per simulation timestep:

1. At current position (for physics calculations)
2. At updated position (after integration step)

For complex terrain implementations (heightmaps, procedural generation):

- **Cache computed values**: If height and normal share calculations, compute together
- **Use spatial acceleration**: Quadtrees or grids for large heightmaps
- **Precompute normals**: Store normals directly rather than computing from gradient

The current implementation queries `getHeight()`, `getNormal()`, and `getSurfaceProperties()` separately, allowing simple implementations while permitting optimization in complex cases.

## Example: Multi-Surface Terrain

```c++
class GolfCourseTerrain : public TerrainInterface
{
public:
    GolfCourseTerrain(const Heightmap& heights) : heightmap_(heights) {}

    float getHeight(float x, float y) const override
    {
        return heightmap_.lookup(x, y);
    }

    Vector3D getNormal(float x, float y) const override
    {
        return heightmap_.computeNormal(x, y);
    }

    const GroundSurface& getSurfaceProperties(float x, float y) const override
    {
        // Different properties for different regions
        if (isInBunker(x, y))
        {
            // (height, restitution, frictionStatic, frictionDynamic, firmness, spinRetention)
            static GroundSurface sand{0.0F, 0.25F, 0.8F, 0.6F, 0.3F, 0.4F};
            return sand;
        }
        else if (isOnGreen(x, y))
        {
            static GroundSurface green{0.0F, 0.5F, 0.4F, 0.15F, 0.9F, 0.85F};
            return green;
        }
        else  // Fairway
        {
            static GroundSurface fairway{0.0F, 0.4F, 0.5F, 0.2F, 0.8F, 0.75F};
            return fairway;
        }
    }

private:
    Heightmap heightmap_;

    bool isInBunker(float x, float y) const
    {
        // Implementation specific to course layout
        return bunkerRegions_.contains(x, y);
    }

    bool isOnGreen(float x, float y) const
    {
        // Implementation specific to course layout
        return greenRegions_.contains(x, y);
    }

    BunkerRegions bunkerRegions_;
    GreenRegions greenRegions_;
};
```

## Migration from Previous Versions

### Backward Compatibility

The terrain system maintains full backward compatibility. Existing code using only `GroundSurface` continues to work:

```c++
// Simple flat ground (no terrain)
FlightSimulator sim(ball, atmos, ground);
```

Internally, a `FlatTerrain` is created automatically from the `GroundSurface` parameter.

### Updating to Terrain Interface

To utilize custom terrain:

```c++
// Create terrain implementation
auto terrain = std::make_shared<YourTerrainImpl>(ground);

// Pass to simulator
FlightSimulator sim(ball, atmos, ground, terrain);
```

The `GroundSurface` parameter serves as a fallback for backward compatibility but is superseded by terrain queries when custom terrain is provided.

## See Also

- **Ground surface properties**: `include/ground_surface.hpp`
- **Terrain interface**: `include/terrain_interface.hpp`
- **Ground physics**: `include/ground_physics.hpp`
- **Physics constants**: `include/physics_constants.hpp`
- **Terrain tests**: `test/test_terrain_interface.cpp`
- **Ground physics tests**: `test/test_ground_physics.cpp`
