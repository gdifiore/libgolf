# Ground Providers

## Overview

The `GroundProvider` interface lets you change ground properties based on position - model fairways, roughs, greens, and other surface variations on flat terrain.

## Choosing Between GroundProvider and TerrainInterface

### GroundProvider

Use for flat terrain with varying materials:
- Fairway, rough, and green with different properties
- Simple API - return surface properties based on position
- Good for upgrading existing code with minimal changes

Limitation: Assumes flat terrain. The surface normal is always vertical.

### TerrainInterface

Use for 3D terrain with elevation:
- Hills, slopes, or elevated greens
- Slope-dependent bounce physics (varying surface normals)
- Full control over terrain geometry

See the [Terrain System Guide](terrain.md) for implementing 3D terrain.

```cpp
class GroundProvider {
public:
    virtual GroundSurface getGroundAt(float x, float y) const = 0;
};
```

**Parameters:**
- `x`: Lateral position in feet (perpendicular to target, positive = right)
- `y`: Downrange position in feet (along target line, positive = forward)

**Returns:** `GroundSurface` struct with physical properties

## Basic Example

```cpp
class SimpleHole : public GroundProvider {
public:
    GroundSurface getGroundAt(float x, float y) const override {
        float yards = y / physics_constants::YARDS_TO_FEET;

        // Green at 250+ yards, elevated 3 feet
        // (height, restitution, frictionStatic, frictionDynamic, firmness, spinRetention)
        if (yards >= 250.0f) {
            return GroundSurface{3.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
        }

        // Fairway with default values
        return GroundSurface{};
    }
};

// Usage
SimpleHole provider;
FlightSimulator sim(ball, atmos, provider);
```

## Ground Surface Parameters

```cpp
GroundSurface{
    height,            // feet (0.0 = ground level)
    restitution,       // bounce coefficient (0.0-1.0)
    frictionStatic,    // impact friction (0.0-1.0+)
    frictionDynamic,   // rolling resistance (0.0-1.0+)
    firmness,          // ground hardness (0.0-1.0+)
    spinRetention      // spin after impact (0.0-1.0)
};
```

**Common presets:**

```cpp
// Fairway
GroundSurface{0.0f, 0.4f, 0.5f, 0.2f, 0.8f, 0.75f};

// Fast green
GroundSurface{0.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};

// Thick rough
GroundSurface{0.0f, 0.25f, 0.6f, 0.5f, 0.4f, 0.55f};

// Sand bunker
GroundSurface{-0.5f, 0.1f, 0.8f, 0.9f, 0.15f, 0.3f};
```

### Parameter Details

**height**: Ground elevation in feet
- `0.0f`: Ground level
- `3.0f`: Elevated green
- `-1.0f`: Sunken bunker

**restitution**: Coefficient of restitution (COR) - energy retained = COR²
- `0.1f`: Sand bunker (retains 1% of energy)
- `0.4f`: Fairway (retains 16% of energy)
- `0.5f`: Hard surface (retains 25% of energy)

**frictionStatic**: Impact friction (combined with firmness)
- `0.3f`: Low friction
- `0.5f`: Fairway
- `0.8f`: Sand

**frictionDynamic**: Rolling resistance - deceleration = friction × gravity
- `0.12f`: Fast green (stimp 12+)
- `0.2f`: Medium fairway
- `0.5f`: Thick rough

**firmness**: Ground hardness - softer ground applies more friction on impact
- `0.15f`: Soft sand
- `0.8f`: Firm fairway
- `0.95f`: Hard green

**spinRetention**: Fraction of spin remaining after impact
- `0.3f`: Sand bunker
- `0.75f`: Fairway
- `0.85f`: Soft green

## Implementation Patterns

### Distance-based

```cpp
GroundSurface getGroundAt(float x, float y) const override {
    float yards = y / physics_constants::YARDS_TO_FEET;
    if (yards >= 250.0f) return greenSurface;
    return fairwaySurface;
}
```

### With lateral rough

```cpp
GroundSurface getGroundAt(float x, float y) const override {
    float lateralYards = std::abs(x) / physics_constants::YARDS_TO_FEET;
    float downrangeYards = y / physics_constants::YARDS_TO_FEET;

    // Green
    if (downrangeYards >= 250.0f && downrangeYards <= 270.0f) {
        return GroundSurface{3.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
    }

    // Rough
    if (lateralYards > 20.0f) {
        return GroundSurface{0.0f, 0.25f, 0.6f, 0.5f, 0.4f, 0.55f};
    }

    // Fairway
    return GroundSurface{};
}
```

### Circular features

```cpp
GroundSurface getGroundAt(float x, float y) const override {
    float dx = x - centerX;
    float dy = y - centerY;
    float distance = std::sqrt(dx*dx + dy*dy);

    if (distance <= radius) return greenSurface;
    return fairwaySurface;
}
```

### Grid-based

```cpp
class GridTerrain : public GroundProvider {
public:
    GridTerrain(const std::vector<std::vector<GroundSurface>>& grid,
                float cellSizeYards)
        : grid_(grid),
          cellSize_(cellSizeYards * physics_constants::YARDS_TO_FEET) {}

    GroundSurface getGroundAt(float x, float y) const override {
        int col = static_cast<int>(x / cellSize_);
        int row = static_cast<int>(y / cellSize_);

        if (row < 0 || row >= grid_.size() ||
            col < 0 || col >= grid_[0].size()) {
            return defaultSurface;
        }

        return grid_[row][col];
    }

private:
    std::vector<std::vector<GroundSurface>> grid_;
    float cellSize_;
    GroundSurface defaultSurface;
};
```

## When Ground is Queried

Ground is checked:

1. Before each bounce (when ball contacts ground while moving downward)
2. Every 0.1 seconds during roll phase
3. At phase transitions (aerial → bounce, bounce → roll)

Your `getGroundAt()` is called 3-10 times per trajectory, not every simulation step (which would be 500+).

This means:
- Each bounce uses the ground properties at that location
- Rolling ball picks up surface changes every 0.1 seconds
- No queries during aerial phase (doesn't matter what's below)


## Performance

- Called 10-30 times per trajectory (not every step)
- Pre-compute data in constructor, not in `getGroundAt()`
- Avoid file I/O or database queries in `getGroundAt()`
- Simple logic is fine - optimization rarely needed

If you have an expensive ground provider (database queries, etc.), cache results based on position.

## Complete Example

See `examples/multi_ground_simulation.cpp`:

```cpp
class GolfHoleProvider : public GroundProvider {
public:
    GroundSurface getGroundAt(float x, float y) const override {
        const float lateralYards = x / physics_constants::YARDS_TO_FEET;
        const float downrangeYards = y / physics_constants::YARDS_TO_FEET;

        // Green: 250-270 yards, elevated 3 feet
        if (downrangeYards >= 250.0f && downrangeYards <= 270.0f) {
            return GroundSurface{3.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
        }

        // Rough: beyond ±20 yards from centerline
        if (std::abs(lateralYards) > 20.0f) {
            return GroundSurface{0.0f, 0.25f, 0.6f, 0.5f, 0.4f, 0.55f};
        }

        // Fairway
        return GroundSurface{0.0f, 0.4f, 0.5f, 0.2f, 0.8f, 0.75f};
    }
};

int main() {
    const LaunchData ball{160.0f, 11.0f, 0.0f, 3000.0f, 0.0f};
    const AtmosphericData atmos{70.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50.0f, 29.92f};

    GolfHoleProvider provider;
    FlightSimulator sim(ball, atmos, provider);
    sim.run();

    LandingResult result = sim.getLandingResult();
    printf("Landed at: %.1f yards\n", result.yF);
}
```

## Coordinate System

- **Origin (0, 0)**: Ball starting position
- **X-axis**: Lateral (negative = left, positive = right)
- **Y-axis**: Downrange (positive = toward target)
- **Units**: Feet (use `physics_constants::YARDS_TO_FEET` for conversion)

```cpp
// Yards to feet
float feet = yards * physics_constants::YARDS_TO_FEET;  // 3.0

// Feet to yards
float yards = feet / physics_constants::YARDS_TO_FEET;
```

## Using a Uniform Ground

If you don't need position-dependent surfaces, pass a `GroundSurface` directly:

```cpp
// Single ground surface (uses default fairway values)
GroundSurface ground;
FlightSimulator sim(ball, atmos, ground);

// Or with custom values
GroundSurface green{0.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
FlightSimulator sim2(ball, atmos, green);
```

Internally creates a `UniformGroundProvider` that returns the same surface everywhere.
