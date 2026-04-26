// Calibration shot runner.
//
// Reads launch+atmos rows from stdin (CSV with header), runs each through
// FlightSimulator, prints per-shot result CSV to stdout.
//
// Input columns (header required, order flexible):
//   shot_id, ball_speed_mph, launch_angle_deg, direction_deg,
//   backspin_rpm, sidespin_rpm, temp_f, elevation_ft, wind_mph,
//   wind_dir_deg, humidity_pct, pressure_inhg
//
// Output columns: shot_id, carry_yd, total_yd, apex_yd, side_yd,
//   bearing_deg, time_s

#include "FlightSimulator.hpp"
#include "atmospheric_data.hpp"
#include "ground_surface.hpp"
#include "launch_data.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> splitCsv(const std::string &line)
{
	std::vector<std::string> out;
	std::stringstream ss(line);
	std::string cell;
	while (std::getline(ss, cell, ','))
	{
		out.push_back(cell);
	}
	return out;
}

float toFloat(const std::string &s, float fallback = 0.0F)
{
	try
	{
		return std::stof(s);
	}
	catch (...)
	{
		return fallback;
	}
}
} // namespace

int main()
{
	std::string header;
	if (!std::getline(std::cin, header))
	{
		std::fprintf(stderr, "sim_runner: empty input\n");
		return 1;
	}

	const auto cols = splitCsv(header);
	std::map<std::string, size_t> idx;
	for (size_t i = 0; i < cols.size(); ++i)
	{
		idx[cols[i]] = i;
	}

	auto col = [&](const std::vector<std::string> &row, const std::string &name, float fallback = 0.0F) {
		auto it = idx.find(name);
		if (it == idx.end() || it->second >= row.size())
		{
			return fallback;
		}
		return toFloat(row[it->second], fallback);
	};
	auto sCol = [&](const std::vector<std::string> &row, const std::string &name) -> std::string {
		auto it = idx.find(name);
		if (it == idx.end() || it->second >= row.size())
		{
			return "";
		}
		return row[it->second];
	};

	std::printf("shot_id,carry_yd,total_yd,apex_yd,side_yd,bearing_deg,time_s\n");

	std::string line;
	while (std::getline(std::cin, line))
	{
		if (line.empty())
		{
			continue;
		}
		const auto row = splitCsv(line);

		const LaunchData launch{
			.ballSpeedMph   = col(row, "ball_speed_mph"),
			.launchAngleDeg = col(row, "launch_angle_deg"),
			.directionDeg   = col(row, "direction_deg"),
			.backspinRpm    = col(row, "backspin_rpm"),
			.sidespinRpm    = col(row, "sidespin_rpm"),
		};
		const AtmosphericData atmos{
			.temp        = col(row, "temp_f", 70.0F),
			.elevation   = col(row, "elevation_ft", 0.0F),
			.vWind       = col(row, "wind_mph", 0.0F),
			.phiWind     = col(row, "wind_dir_deg", 0.0F),
			.hWind       = 0.0F,
			.relHumidity = col(row, "humidity_pct", 50.0F),
			.pressure    = col(row, "pressure_inhg", 29.92F),
		};
		const GroundSurface ground;

		FlightSimulator sim(launch, atmos, ground);
		auto traj = sim.runAndGetTrajectory();

		const float ydPerFt = 1.0F / physics_constants::YARDS_TO_FEET;
		float apexFt = 0.0F;
		size_t carryIdx = 0;
		bool airborneSeen = false;

		for (size_t i = 0; i < traj.size(); ++i)
		{
			const float z = traj[i].position[2];
			apexFt = std::max(apexFt, z);
			if (z > 0.5F)
			{
				airborneSeen = true;
			}
			else if (airborneSeen && carryIdx == 0)
			{
				carryIdx = i; // first ground touch after going up
			}
		}
		if (carryIdx == 0 && !traj.empty())
		{
			carryIdx = traj.size() - 1;
		}

		const auto &carryState = traj[carryIdx];
		const float carryYd = std::sqrt(carryState.position[0] * carryState.position[0] +
		                                carryState.position[1] * carryState.position[1]) *
		                      ydPerFt;

		const LandingResult lr = sim.getLandingResult();
		const float totalYd = std::sqrt(lr.xF * lr.xF + lr.yF * lr.yF);

		std::printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f\n",
		            sCol(row, "shot_id").c_str(),
		            carryYd,
		            totalYd,
		            apexFt * ydPerFt,
		            lr.xF,
		            lr.bearing,
		            lr.timeOfFlight);
	}
	return 0;
}
