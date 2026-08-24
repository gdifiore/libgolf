#include <gtest/gtest.h>

#include <libgolf.hpp>

TEST(GolfTest, FtoC)
{
    float tempC = math_utils::convertFahrenheitToCelsius(32.0);

    EXPECT_NEAR(tempC, 0.0, 0.1);
}

TEST(GolfTest, CtoK)
{
    float tempK = math_utils::convertCelsiusToKelvin(32.0);

    EXPECT_NEAR(tempK, 305.15, 0.1);
}

TEST(GolfTest, FtoK)
{
    float tempK = math_utils::convertFahrenheitToKelvin(32.0);

    EXPECT_NEAR(tempK, 273.16, 0.1);
}

TEST(GolfTest, FTtoM)
{
    float tempK = math_utils::convertFeetToMeters(5.0);

    EXPECT_NEAR(tempK, 1.524, 0.1);
}

TEST(GolfTest, MtoFT)
{
    float lengthFT = math_utils::convertMetersToFeet(10.0);

    EXPECT_NEAR(lengthFT, 32.8084, 0.01);
}

TEST(GolfTest, DistanceInYards)
{
    Vector3D position{3.0, 4.0, 0.0};
    float distanceYards = math_utils::getDistanceInYards(position);

    EXPECT_NEAR(distanceYards, 1.66, 0.01);
}

// Vector Math Tests

TEST(GolfTest, DotProduct)
{
    Vector3D a{1.0, 0.0, 0.0};
    Vector3D b{0.0, 1.0, 0.0};
    float result = math_utils::dot(a, b);

    EXPECT_NEAR(result, 0.0, 0.001);

    Vector3D c{1.0, 2.0, 3.0};
    Vector3D d{4.0, 5.0, 6.0};
    result = math_utils::dot(c, d);

    EXPECT_NEAR(result, 32.0, 0.001); // 1*4 + 2*5 + 3*6 = 32
}

TEST(GolfTest, CrossProduct)
{
    Vector3D x{1.0, 0.0, 0.0};
    Vector3D y{0.0, 1.0, 0.0};
    Vector3D result = math_utils::cross(x, y);

    EXPECT_NEAR(result[0], 0.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 1.0, 0.001); // x × y = z

    Vector3D a{1.0, 2.0, 3.0};
    Vector3D b{4.0, 5.0, 6.0};
    result = math_utils::cross(a, b);

    EXPECT_NEAR(result[0], -3.0, 0.001); // 2*6 - 3*5 = -3
    EXPECT_NEAR(result[1], 6.0, 0.001);  // 3*4 - 1*6 = 6
    EXPECT_NEAR(result[2], -3.0, 0.001); // 1*5 - 2*4 = -3
}

TEST(GolfTest, Magnitude)
{
    Vector3D v{3.0, 4.0, 0.0};
    float mag = math_utils::magnitude(v);

    EXPECT_NEAR(mag, 5.0, 0.001); // 3-4-5 triangle

    Vector3D v2{1.0, 1.0, 1.0};
    mag = math_utils::magnitude(v2);

    EXPECT_NEAR(mag, 1.732, 0.001); // sqrt(3)
}

TEST(GolfTest, Normalize)
{
    Vector3D v{3.0, 4.0, 0.0};
    Vector3D normalized = math_utils::normalize(v);

    EXPECT_NEAR(normalized[0], 0.6, 0.001); // 3/5
    EXPECT_NEAR(normalized[1], 0.8, 0.001); // 4/5
    EXPECT_NEAR(normalized[2], 0.0, 0.001);

    // Verify unit length
    float mag = math_utils::magnitude(normalized);
    EXPECT_NEAR(mag, 1.0, 0.001);
}

TEST(GolfTest, NormalizeZeroVector)
{
    Vector3D zero{0.0, 0.0, 0.0};

    EXPECT_THROW(math_utils::normalize(zero), std::invalid_argument);
}

TEST(GolfTest, VectorProjection)
{
    // Project (3, 4, 0) onto (1, 0, 0)
    Vector3D v{3.0, 4.0, 0.0};
    Vector3D onto{1.0, 0.0, 0.0};
    Vector3D projection = math_utils::project(v, onto);

    EXPECT_NEAR(projection[0], 3.0, 0.001);
    EXPECT_NEAR(projection[1], 0.0, 0.001);
    EXPECT_NEAR(projection[2], 0.0, 0.001);

    // Project (1, 1, 0) onto (1, 1, 0) - should be itself
    Vector3D v2{1.0, 1.0, 0.0};
    Vector3D onto2{1.0, 1.0, 0.0};
    projection = math_utils::project(v2, onto2);

    EXPECT_NEAR(projection[0], 1.0, 0.001);
    EXPECT_NEAR(projection[1], 1.0, 0.001);
    EXPECT_NEAR(projection[2], 0.0, 0.001);
}

TEST(GolfTest, ProjectOntoZeroVector)
{
    Vector3D v{1.0, 2.0, 3.0};
    Vector3D zero{0.0, 0.0, 0.0};

    EXPECT_THROW(math_utils::project(v, zero), std::invalid_argument);
}
