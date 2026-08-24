#ifndef CALIBRATED_AERODYNAMIC_MODEL_HPP
#define CALIBRATED_AERODYNAMIC_MODEL_HPP

#include "AerodynamicModel.hpp"
#include "DefaultAerodynamicModel.hpp"

#include <cmath>
#include <stdexcept>

/**
 * @brief Small, interpretable calibration layer over DefaultAerodynamicModel.
 *
 * This model preserves the published Reynolds/spin coefficient curves and
 * only permits three physically separable corrections:
 *
 * - `dragScale` scales Cd;
 * - `liftScale` scales Cl; and
 * - `spinDecayScale` scales the aerial spin-decay time constant.
 *
 * It is intentionally not the simulator default. The public Garmin R50 set
 * has no ball or club identifier, no wind, and no ground metadata, so its fit
 * describes its own calm-condition measurement population rather than a
 * universal replacement for the reference aerodynamics.
 */
struct AerodynamicCalibration
{
	float dragScale = 1.0F;
	float liftScale = 1.0F;
	float spinDecayScale = 1.0F;
};

class CalibratedAerodynamicModel final : public AerodynamicModel
{
public:
	/**
	 * @brief Garmin-R50 fit from `tools/fitting/fit_aero.py`.
	 *
	 * This is a robust 90%/10% train/validation fit to 800 aerial-eligible
	 * rows of the public dataset. It remains a model selection, not a
	 * device-calibration claim.
	 */
	[[nodiscard]] static constexpr AerodynamicCalibration garminR50Fit()
	{
		return {.dragScale = 0.88275F, .liftScale = 0.99205F, .spinDecayScale = 1.59676F};
	}

	explicit CalibratedAerodynamicModel(AerodynamicCalibration calibration = garminR50Fit())
		: calibration_(calibration)
	{
		if (!isValidScale(calibration_.dragScale) ||
		    !isValidScale(calibration_.liftScale) ||
		    !isValidScale(calibration_.spinDecayScale))
		{
			throw std::invalid_argument("Aerodynamic calibration scales must be finite and positive");
		}
	}

	[[nodiscard]] Vector3D computeAcceleration(const AerodynamicState &state) const override
	{
		return reference_.computeAccelerationScaled(
			state, calibration_.dragScale, calibration_.liftScale);
	}

	[[nodiscard]] float computeSpinDecayTau(const AerodynamicState &state) const override
	{
		return reference_.computeSpinDecayTauScaled(state, calibration_.spinDecayScale);
	}

	[[nodiscard]] const AerodynamicCalibration &calibration() const { return calibration_; }

private:
	[[nodiscard]] static bool isValidScale(float scale)
	{
		return std::isfinite(scale) && scale > 0.0F;
	}

	AerodynamicCalibration calibration_;
	DefaultAerodynamicModel reference_;
};

#endif // CALIBRATED_AERODYNAMIC_MODEL_HPP
