"""
CanSat Seed Dispersal Landing Zone Calculator

This program calculates where a seed dropped from a CanSat will land,
accounting for wind drift, terminal velocity, and atmospheric conditions.
"""

import math
from typing import List, Optional, Dict, Any, Tuple


# Constants
G = 9.81  # [m/s^2] acceleration due to gravity
R_AIR = 287.058  # [J/(kg·K)] gas constant for dry air
R_EARTH = 6_371_000  # [m] mean Earth radius


class GPSSample:
    """Structure to hold GPS data point"""
    def __init__(self, lat: float, lon: float, alt: float, time: float):
        self.lat = lat  # [degrees]
        self.lon = lon  # [degrees]
        self.alt = alt  # [m]
        self.time = time  # [s]


def calculate_wind_at_height(gps_samples: List[GPSSample], target_height: float = None) -> Tuple[float, float, float, float]:
    """
    Calculate horizontal wind speed and direction using linear regression on GPS data.
    Uses all GPS samples in the window to reduce position noise through least-squares fitting.
    
    Args:
        gps_samples: List of GPS samples (typically 10s window at 1 Hz = ~11 points)
        target_height: Release height H [m] (optional, for reference)
    
    Returns:
        Tuple of (U_x, U_y, U_H, theta)
        - U_x [m/s]: east component of wind
        - U_y [m/s]: north component of wind
        - U_H [m/s]: wind speed at height H
        - theta [rad]: direction wind blows toward (0 = north, π/2 = east)
    """
    if len(gps_samples) < 2:
        raise ValueError("Need at least 2 GPS samples to calculate wind")
    
    # Use first point as reference (origin of local coordinate system)
    ref_point = gps_samples[0]
    ref_phi = ref_point.lat * math.pi / 180  # [rad]
    ref_time = ref_point.time
    
    # Convert all GPS points to local Cartesian coordinates (East, North) in meters
    times = []
    x_coords = []  # East [m]
    y_coords = []  # North [m]
    
    for point in gps_samples:
        # Time relative to first point
        t = point.time - ref_time
        
        # Convert lat/lon differences to meters (ENU frame)
        delta_phi = (point.lat - ref_point.lat) * math.pi / 180  # [rad]
        delta_lambda = (point.lon - ref_point.lon) * math.pi / 180  # [rad]
        
        # Local flat-earth approximation
        x = R_EARTH * math.cos(ref_phi) * delta_lambda  # [m] East
        y = R_EARTH * delta_phi  # [m] North
        
        times.append(t)
        x_coords.append(x)
        y_coords.append(y)
    
    # Calculate means
    n = len(times)
    t_mean = sum(times) / n
    x_mean = sum(x_coords) / n
    y_mean = sum(y_coords) / n
    
    # Linear regression: fit x(t) = a_x + v_x*t, y(t) = a_y + v_y*t
    # Slope formula: v = Σ(t_i - t_mean)(coord_i - coord_mean) / Σ(t_i - t_mean)²
    
    sum_t_dev_sq = sum((t - t_mean)**2 for t in times)
    
    if sum_t_dev_sq == 0:
        raise ValueError("All time samples are identical - cannot compute velocity")
    
    # Calculate v_x (eastward velocity)
    sum_tx = sum((times[i] - t_mean) * (x_coords[i] - x_mean) for i in range(n))
    v_x = sum_tx / sum_t_dev_sq
    
    # Calculate v_y (northward velocity)
    sum_ty = sum((times[i] - t_mean) * (y_coords[i] - y_mean) for i in range(n))
    v_y = sum_ty / sum_t_dev_sq
    
    # Wind speed and direction
    U_H = math.sqrt(v_x**2 + v_y**2)  # [m/s] wind speed
    theta = math.atan2(v_x, v_y)  # [rad] direction (0 = north, π/2 = east)
    
    return v_x, v_y, U_H, theta


def wind_speed_at_height(z: float, U_H: float, H: float, z_0: float, d: float = 0.0) -> float:
    """
    Calculate wind speed at height z using logarithmic wind profile.
    
    PHYSICAL MODEL:
    ---------------
    For open terrain (d = 0):
        U(z) = U(H) * ln(z/z_0) / ln(H/z_0)
    
    For forested/canopy terrain (d > 0):
        U(z) = U(H) * ln((z-d)/z_0) / ln((H-d)/z_0)
    
    where:
        - z_0 is the roughness length [m]
        - d is the zero-plane displacement height [m]
    
    PARAMETERS:
    -----------
    z : float
        Height above ground where wind speed is desired [m]
        Must satisfy: z > d + z_0 for valid result
    
    U_H : float
        Wind speed at reference height H [m/s]
    
    H : float
        Reference height where U_H was measured [m]
        Must satisfy: H > d + z_0
    
    z_0 : float
        Roughness length [m]
        Typical values:
            - Sand, desert, smooth terrain: 0.001 - 0.005 m
            - Short grass: 0.01 - 0.04 m
            - Crops, hedges: 0.1 - 0.25 m
            - Forest: 0.5 - 1.0 m
    
    d : float, optional (default=0.0)
        Zero-plane displacement height [m]
        
        Physical meaning: The effective height at which momentum absorption 
        occurs in a canopy. Represents the mean level of momentum sink in 
        vegetated terrain.
        
        When to use:
            - d = 0: Open terrain (desert, water, short grass, bare soil)
            - d > 0: Forests, tall crops, dense vegetation, urban canopies
        
        Typical estimation:
            - Open/sparse vegetation: d = 0 m
            - Crops/bushes: d ≈ 0.6-0.7 × vegetation height
            - Dense forest: d ≈ 0.7-0.8 × canopy height
        
        Examples:
            - Desert: d = 0 m
            - 3m tall crops: d ≈ 2 m
            - 20m forest canopy: d ≈ 14-16 m
    
    RETURNS:
    --------
    float
        Wind speed at height z [m/s]
        Returns 0.0 if z is below the valid profile range (z ≤ d + z_0)
    
    VALIDITY RANGE:
    ---------------
    This logarithmic profile model is valid in the atmospheric surface layer 
    (typically 10-100m) under the following assumptions:
    
    1. Neutral atmospheric stability (no strong heating/cooling)
    2. Horizontal homogeneous terrain (upwind fetch > 100× canopy height)
    3. Steady-state flow
    4. Height well above roughness sublayer: z > d + 2*z_0 (recommended)
    5. Below boundary layer top: z < 100-200m (typically)
    
    For z < d + 2*z_0, wake effects and canopy mixing may invalidate the 
    logarithmic assumption. For z > 200m, thermal stratification and 
    Coriolis effects become important.
    
    USER RESPONSIBILITY:
    --------------------
    The user must provide appropriate z_0 and d based on:
        - GPS location at measurement height
        - Wind direction (determines upwind terrain)
        - Local terrain survey or lookup tables
    
    EXAMPLES:
    ---------
    # Example 1: Open desert terrain (Błędowska Desert)
    >>> U_50m = wind_speed_at_height(z=50, U_H=10.0, H=100, z_0=0.002, d=0.0)
    >>> # Returns wind at 50m using classic log law
    
    # Example 2: Forest with 20m canopy height
    >>> U_50m = wind_speed_at_height(z=50, U_H=10.0, H=100, z_0=1.0, d=14.0)
    >>> # Returns wind at 50m with displacement correction
    
    # Example 3: Backward compatible (original function call)
    >>> U_50m = wind_speed_at_height(z=50, U_H=10.0, H=100, z_0=0.002)
    >>> # d defaults to 0.0, behaves as original function
    
    RAISES:
    -------
    ValueError
        - If d < 0 (displacement must be non-negative)
        - If H ≤ d (reference height must be above displacement)
        - If H - d ≤ z_0 (reference height invalid for log profile)
    """
    # Validate displacement height
    if d < 0:
        raise ValueError(f"Displacement height d must be non-negative, got d={d}")
    
    # Check reference height validity
    if H <= d:
        raise ValueError(
            f"Reference height H must be above displacement height d.\n"
            f"Got H={H} m, d={d} m.\n"
            f"For forest canopy, ensure H > canopy top."
        )
    
    # Effective heights above displacement level
    effective_z = z - d
    effective_H = H - d
    
    # Smooth, differentiable handling near/inside roughness layer
    # We want a strictly positive log argument and (optionally) non-negative wind speed.
    # Use a smooth approximation of max(effective_z, z_0) to avoid hard clamps (derivative-friendly).
    epsilon = z_0 / 10.0
    smooth_effective_z = 0.5 * (effective_z + z_0 + math.sqrt((effective_z - z_0) ** 2 + epsilon ** 2))
# Validate reference height is above roughness layer
    if effective_H <= z_0:
        raise ValueError(
            f"Reference height (H-d) must be above roughness length z_0.\n"
            f"Got H-d={effective_H:.3f} m, z_0={z_0} m.\n"
            f"Check your displacement height or reference height values."
        )
    
    # Compute displacement-corrected logarithmic wind profile
    # U(z) = U(H) * ln((z-d)/z_0) / ln((H-d)/z_0)
    U_z = U_H * math.log(smooth_effective_z / z_0) / math.log(effective_H / z_0)
    
    return U_z


def calculate_air_density(pressure: float, temperature: float) -> float:
    """
    Calculate air density from pressure and temperature.
    
    Equation (4): ρ = p / (R * T)
    
    Args:
        pressure: Air pressure [Pa]
        temperature: Air temperature [K]
    
    Returns:
        Air density [kg/m³]
    """
    return pressure / (R_AIR * temperature)


def calculate_terminal_velocity(mass: float, area: float, C_d: float, rho: float) -> float:
    """
    Calculate terminal velocity of the seed.
    
    Equation (5): v_t = sqrt(2*m*g / (ρ*C_d*A))
    
    Args:
        mass: Seed mass [kg]
        area: Projected area [m²]
        C_d: Drag coefficient [-]
        rho: Air density [kg/m³]
    
    Returns:
        Terminal velocity [m/s]
    """
    return math.sqrt(2 * mass * G / (rho * C_d * area))


def calculate_fall_time(height: float, v_t: float) -> float:
    """
    Calculate time for seed to fall from height H to ground.
    
    Equation (7): t_f ≈ H/v_t + (v_t/g)*ln(2)
    
    Args:
        height: Release height [m]
        v_t: Terminal velocity [m/s]
    
    Returns:
        Fall time [s]
    """
    cruise_time = height / v_t
    correction = (v_t / G) * math.log(2)
    
    return cruise_time + correction


def calculate_horizontal_drift_surface_layer(
    U_H: float,
    v_t: float,
    H: float,
    z_0: float,
    d: float = 0.0,
    z_g: float = 1.0,
    n_steps: int = 200
) -> float:
    """
    Calculate horizontal drift in the atmospheric surface layer (≈ 0 - 100 m AGL)
    by integrating the wind profile over height.

    PHYSICAL MODEL:
    ---------------
    Horizontal drift is computed as:

        D = ∫[z_g → H] U(z) / v_t dz

    where:
        - U(z) is given by wind_speed_at_height(...)
        - v_t is the (assumed constant) terminal velocity

    This formulation:
        - Naturally incorporates both sand and forest wind models
        - Respects surface-layer validity of the logarithmic profile
        - Is smooth and derivative-consistent by construction

    PARAMETERS:
    -----------
    U_H : float
        Wind speed at reference height H [m/s] (typically H ≈ 100 m)

    v_t : float
        Terminal vertical velocity of the capsule [m/s]

    H : float
        Upper integration height [m] (should be ≤ 100 m)

    z_0 : float
        Roughness length [m]

    d : float, optional
        Zero-plane displacement height [m]
        d = 0   → sand / open terrain
        d > 0   → forest / canopy terrain

    z_g : float, optional
        Lower cutoff height [m] to avoid near-ground singular behavior
        (default: 1 m)

    n_steps : int, optional
        Number of integration steps (default: 200)

    RETURNS:
    --------
    float
        Horizontal drift distance in the surface layer [m]

    VALIDITY:
    ---------
    - Assumes near-terminal vertical descent
    - Assumes neutral atmospheric stability
    - Valid primarily for 0 - 100 m AGL
    """

    if z_g <= 0:
        raise ValueError("Ground cutoff height z_g must be positive")

    if H <= z_g:
        raise ValueError("Upper height H must be greater than z_g")

    dz = (H - z_g) / n_steps
    drift = 0.0

    for i in range(n_steps):
        # Midpoint rule for better accuracy and smoothness
        z_mid = z_g + (i + 0.5) * dz

        U_z = wind_speed_at_height(
            z=z_mid,
            U_H=U_H,
            H=H,
            z_0=z_0,
            d=d
        )

        drift += U_z * dz / v_t

    return drift

def calculate_landing_position(
    release_lat: float,
    release_lon: float,
    drift_x: float,
    drift_y: float
) -> Tuple[float, float]:
    """
    Calculate landing position from release position and horizontal drift.

    Uses a local tangent-plane (ENU) approximation, valid for small distances
    (≲ several kilometers), which is fully sufficient for CanSat applications.

    Args:
        release_lat : float
            Release latitude [degrees]
        release_lon : float
            Release longitude [degrees]
        drift_x : float
            Eastward drift [m]
        drift_y : float
            Northward drift [m]

    Returns:
        (landing_lat, landing_lon) : tuple of floats
            Landing latitude and longitude [degrees]
    """
    # Convert latitude to radians
    phi = math.radians(release_lat)

    # Convert meter offsets to angular offsets (radians)
    delta_lat = drift_y / R_EARTH
    delta_lon = drift_x / (R_EARTH * math.cos(phi))

    # Convert back to degrees
    landing_lat = release_lat + math.degrees(delta_lat)
    landing_lon = release_lon + math.degrees(delta_lon)

    return landing_lat, landing_lon


def calculate_cansat_landing(
    H: float,
    m: float,
    A: float,
    C_d: float,
    pressure: float,
    temperature_celsius: float,
    gps_samples: List["GPSSample"],
    z_0: float = 0.002,
    d: float = 0.0,
    z_surface: float = 100.0,
    z_g: float = 1.0,
    release_lat: Optional[float] = None,
    release_lon: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Calculate CanSat landing position using a physically consistent two-regime drift model.

    MODEL OVERVIEW
    --------------
    Total horizontal drift is split into two parts:

    1) High-altitude drift (release height H down to z_surface):
       - Uses GPS-derived wind vector (assumed representative above z_surface)
       - Drift_high_vector = wind_vector * t_high
       - t_high ≈ (H - z_surface) / v_t  (terminal descent approximation)

    2) Surface-layer drift (z_surface down to ground cutoff z_g):
       - Uses logarithmic wind profile (valid in surface layer) and integrates:
         D_surface = ∫_{z_g}^{z_surface} U(z)/v_t dz
       - Uses roughness z_0 and displacement d (forest support)
       - Returns vector drift using the wind direction at z_surface (assumed constant through surface layer)

    VALIDITY / ASSUMPTIONS
    ----------------------
    - Neutral stability (no strong thermal stratification)
    - Terminal vertical speed approximation (v_t constant)
    - Wind direction is treated as constant within each regime
    - Log wind profile is ONLY used for z <= z_surface

    Parameters
    ----------
    H : release height above ground [m]
    d : zero-plane displacement height [m]
        - d = 0 -> sand/open terrain
        - d > 0 -> forest/canopy terrain
    z_surface : top of surface layer for log-profile integration [m] (default 100)
    z_g : near-ground cutoff [m] (default 1)

    Returns
    -------
    dict with:
      - landing_lat, landing_lon
      - drift_vector: {"east": ..., "north": ...}
      - drift_distance (magnitude)
      - high_altitude_drift: {"east","north","magnitude","time"}
      - surface_layer_drift: {"east","north","magnitude"}
      - fall_time, terminal_velocity, air_density
      - wind_release: {"east","north","speed","theta_rad"}
      - wind_at_z_surface: {"speed","theta_rad"}
    """

    # ----------------------------
    # Basic input validation
    # ----------------------------
    if H <= 0:
        raise ValueError(f"Release height H must be positive, got H={H}")
    if not gps_samples or len(gps_samples) < 2:
        raise ValueError("gps_samples must contain at least 2 samples to estimate wind")
    if z_surface <= 0:
        raise ValueError(f"z_surface must be positive, got z_surface={z_surface}")
    if z_g <= 0:
        raise ValueError(f"z_g must be positive, got z_g={z_g}")
    if z_g >= z_surface:
        raise ValueError(f"z_g must be < z_surface, got z_g={z_g}, z_surface={z_surface}")
    if d < 0:
        raise ValueError(f"Displacement height d must be non-negative, got d={d}")
    if z_0 <= 0:
        raise ValueError(f"Roughness length z_0 must be positive, got z_0={z_0}")

    # Use first GPS sample as release position if not specified
    if release_lat is None:
        release_lat = gps_samples[0].lat
    if release_lon is None:
        release_lon = gps_samples[0].lon

    # ----------------------------
    # Atmosphere & descent params
    # ----------------------------
    T = temperature_celsius + 273.15  # [K]
    rho = calculate_air_density(pressure, T)
    v_t = calculate_terminal_velocity(m, A, C_d, rho)
    t_f = calculate_fall_time(H, v_t)

    # ----------------------------
    # Wind vector from GPS samples
    # ----------------------------
    # Returns: U_x (east), U_y (north), U_H (speed), theta (direction "to": 0=north, pi/2=east)
    U_x, U_y, U_H, theta = calculate_wind_at_height(gps_samples, target_height=None)

    # ----------------------------
    # Regime split: high-altitude vs surface layer
    # ----------------------------
    # If release height is below surface-layer top, there is no "high-altitude" portion.
    H_top_surface = min(H, z_surface)

    # High-altitude time and drift (release -> z_surface)
    if H > z_surface:
        t_high = (H - z_surface) / v_t
        drift_high_e = U_x * t_high
        drift_high_n = U_y * t_high
    else:
        t_high = 0.0
        drift_high_e = 0.0
        drift_high_n = 0.0

    drift_high_mag = math.hypot(drift_high_e, drift_high_n)

    # ----------------------------
    # Surface-layer wind speed at z_surface
    # ----------------------------
    # We need U(z_surface) to feed surface-layer drift integration.
    # We must choose a reference height for the log scaling.
    # Best: use the height at which U_H was measured. If you do not have altitude in GPSSample,
    # assume gps_samples represent wind at the release height H (common in CanSat use).
    #
    # If your GPSSample has an altitude field, you should replace this block with
    # something like: H_ref = median(sample.alt_agl for sample in gps_samples)
    # Estimate reference height of wind measurement from GPS samples (median altitude).
    # NOTE: This assumes GPSSample.alt is altitude above ground level (AGL). If it's ASL, convert before calling.
    altitudes = [s.alt for s in gps_samples]
    H_ref = sorted(altitudes)[len(altitudes) // 2]

    # Determine wind speed at the top of the surface layer (H_top_surface) for surface-layer integration.
    #
    # If the GPS-derived wind was measured within the surface layer (H_ref <= z_surface),
    # we can scale it to H_top_surface using the log profile (still within validity range).
    #
    # If H_ref is above the surface layer (H_ref > z_surface), it is NOT physically valid to
    # "log-scale" down from that height. In that case, we fall back to assuming the measured wind
    # equals the wind at the surface-layer top, and we report this assumption in the output.
    used_fallback_surface_wind = False

    if H_ref <= z_surface:
        U_at_surface = wind_speed_at_height(
            z=H_top_surface,
            U_H=U_H,
            H=H_ref,
            z_0=z_0,
            d=d
        )
    else:
        U_at_surface = U_H
        used_fallback_surface_wind = True

    # Direction in surface layer is assumed the same as theta from GPS wind vector
    theta_surface = theta

    # ----------------------------
    # Surface-layer drift magnitude and vector (z_surface -> ground)
    # ----------------------------
    # Use surface-layer integral model ONLY up to z_surface
    drift_surface_mag = calculate_horizontal_drift_surface_layer(
        U_H=U_at_surface,
        v_t=v_t,
        H=H_top_surface,   # <= z_surface
        z_0=z_0,
        d=d,
        z_g=z_g,
        n_steps=200
    )

    drift_surface_e = drift_surface_mag * math.sin(theta_surface)
    drift_surface_n = drift_surface_mag * math.cos(theta_surface)

    # ----------------------------
    # Total drift vector
    # ----------------------------
    drift_total_e = drift_high_e + drift_surface_e
    drift_total_n = drift_high_n + drift_surface_n
    drift_total_mag = math.hypot(drift_total_e, drift_total_n)

    # ----------------------------
    # Landing coordinates
    # ----------------------------
    landing_lat, landing_lon = calculate_landing_position(
        release_lat, release_lon, drift_total_e, drift_total_n
    )

    drift_direction_deg = (theta * 180.0 / math.pi) % 360.0  # drift "to" direction

    return {
        "landing_lat": landing_lat,
        "landing_lon": landing_lon,

        "drift_vector": {"east": drift_total_e, "north": drift_total_n},
        "drift_distance": drift_total_mag,
        "drift_direction": drift_direction_deg,

        "fall_time": t_f,
        "terminal_velocity": v_t,
        "air_density": rho,

        "high_altitude_drift": {
            "east": drift_high_e,
            "north": drift_high_n,
            "magnitude": drift_high_mag,
            "time": t_high,
            "z_from": H,
            "z_to": z_surface if H > z_surface else H,
        },

        "surface_layer_drift": {
            "east": drift_surface_e,
            "north": drift_surface_n,
            "magnitude": drift_surface_mag,
            "z_from": H_top_surface,
            "z_to": z_g,
            "z_surface": z_surface,
            "z_0": z_0,
            "d": d,
        },

        "wind_release": {
            "east": U_x,
            "north": U_y,
            "speed": U_H,
            "theta_rad": theta,
        },

        "wind_at_z_surface": {
            "speed": U_at_surface,
            "theta_rad": theta_surface,
        },

        "assumptions": {
            "gps_altitude_assumed_agl": True,
            "high_altitude_wind_constant": True,
            "surface_wind_from_high_altitude_fallback": used_fallback_surface_wind,
            "H_ref_median_gps_altitude": H_ref,
        },
    }


# Example usage
if __name__ == "__main__":
    # Example inputs - CanSat dropping from 100m near Warsaw, Poland
    # Location: Błędowska Desert area (desert roughness)
    H = 100.0  # [m] release height
    m = 0.001  # [kg] seed mass (1 gram)
    A = 0.0001  # [m²] projected area (1 cm²)
    C_d = 0.5  # [-] drag coefficient
    pressure = 101200  # [Pa] typical Warsaw atmospheric pressure
    temperature = 15.0  # [°C] typical spring/fall temperature
    z_0 = 0.002  # [m] roughness length for Błędowska desert
    
    # GPS samples near Warsaw, Poland
    # Coordinates: 52.23°N, 21.01°E
    # Wind speed: 5 m/s with mainly eastward component (typical moderate wind)
    # Time in seconds, positions in degrees, altitude in meters
    # These samples simulate realistic wind drift:
    # At latitude 52.23°: 1° lon ≈ 68,127 m, 1° lat ≈ 111,320 m
    # For 5 m/s east and 1 m/s north wind:
    gps_samples = [
        GPSSample(lat=52.230000, lon=21.010000, alt=100.0, time=0.0),
        GPSSample(lat=52.230009, lon=21.010073, alt=99.8, time=1.0)
    ]
    
    # Calculate landing position
    result = calculate_cansat_landing(
        H=H,
        m=m,
        A=A,
        C_d=C_d,
        pressure=pressure,
        temperature_celsius=temperature,
        gps_samples=gps_samples,
        z_0=z_0
    )
    
    # Print results
    print("=" * 60)
    print("CanSat Seed Landing Prediction - Warsaw, Poland")
    print("Location: Błędowska Desert (desert roughness)")
    print("=" * 60)
    print(f"\nInput Parameters:")
    print(f"  Release height: {H:.1f} m")
    print(f"  Seed mass: {m*1000:.2f} g")
    print(f"  Projected area: {A*10000:.2f} cm²")
    print(f"  Drag coefficient: {C_d:.2f}")
    print(f"  Air pressure: {pressure:.0f} Pa")
    print(f"  Temperature: {temperature:.1f} °C")
    print(f"  Roughness length: {z_0*1000:.1f} mm")
    
    print(f"\nCalculated Values:")
    print(f"  Air density: {result['air_density']:.3f} kg/m³")
    print(f"  Terminal velocity: {result['terminal_velocity']:.2f} m/s")
    print(f"  Wind speed at height: {result['wind_speed']:.2f} m/s")
    print(f"  Wind direction: {result['drift_direction']:.1f}° from north")
    print(f"  Fall time: {result['fall_time']:.2f} s")
    
    print(f"\nLanding Position:")
    print(f"  Release: ({gps_samples[0].lat:.6f}°, {gps_samples[0].lon:.6f}°)")
    print(f"  Landing: ({result['landing_lat']:.6f}°, {result['landing_lon']:.6f}°)")
    print(f"  Horizontal drift: {result['drift_distance']:.2f} m")
    print(f"  Drift direction: {result['drift_direction']:.1f}° from north")
    print("=" * 60)