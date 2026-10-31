"""
Test 7: Communication channel models (Layer 0).

Verifies that each implemented channel model (free-space, log-distance, 3G/4G
cellular, 5G NR, mmWave, fiber) produces a well-formed ``ChannelState`` and
that the physics of path loss and link budget behave as expected.

These are regression tests: they assert the *structure* of the output and a
 handful of physically meaningful invariants (monotonicity, expected dB
 magnitudes). They do not assert exact statistical samples, which would be
 brittle.
"""

import sys
import os
import math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from comm import (
    Pose, RadioParameters,
    FreeSpaceChannel, LogDistanceChannel,
    CellularChannel, NRChannel, MmWaveChannel, FiberChannel,
    THzChannel, FSOChannel,
    create_channel, list_channels,
    ber_for, get_modulation, list_modulations,
    thermal_noise_dbm, snr_db, shannon_capacity_bps,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _pose_at(x, y=0.0, z=0.0, vx=0.0):
    return Pose(
        position=np.array([float(x), float(y), float(z)]),
        velocity=np.array([float(vx), 0.0, 0.0]),
    )


def _assert_state_well_formed(state, tx_id=None, rx_id=None):
    """All scalar fields present, finite where they should be."""
    if tx_id is not None:
        assert state.tx_id == tx_id
    if rx_id is not None:
        assert state.rx_id == rx_id
    assert isinstance(state.path_gain_db, float)
    assert isinstance(state.distance_m, float)
    assert state.distance_m >= 0.0
    assert state.delay_ms >= 0.0
    assert isinstance(state.snr_db, float)
    assert isinstance(state.capacity_bps, float)
    assert state.capacity_bps >= 0.0
    assert 0.0 <= state.ber <= 0.5


# ---------------------------------------------------------------------------
# Free-space
# ---------------------------------------------------------------------------

def test_free_space_basic():
    """Friis: path loss grows with 20 log10(d)."""
    ch = FreeSpaceChannel(carrier_hz=2.4e9)
    tx = _pose_at(0)
    for d in (1.0, 10.0, 100.0):
        st = ch.generate(tx, _pose_at(d), tx_id="a", rx_id="b")
        _assert_state_well_formed(st)
        assert st.los is True
    # Monotonic: farther => more loss (smaller path_gain_db)
    g1 = ch.generate(tx, _pose_at(1.0)).path_gain_db
    g10 = ch.generate(tx, _pose_at(10.0)).path_gain_db
    g100 = ch.generate(tx, _pose_at(100.0)).path_gain_db
    assert g1 > g10 > g100, f"Expected monotonic loss, got {g1},{g10},{g100}"
    # 10x distance => 20 dB more loss (Friis)
    assert abs((g1 - g10) - 20.0) < 0.5, f"Expected ~20 dB per decade, got {g1 - g10}"
    print(f"  [PASS] free_space FSPL: 1m={g1:.1f}, 10m={g10:.1f}, 100m={g100:.1f} dB")


def test_free_space_frequency_scaling():
    """Doubling carrier frequency adds 6 dB path loss at fixed distance."""
    ch_low = FreeSpaceChannel(carrier_hz=2.0e9)
    ch_high = FreeSpaceChannel(carrier_hz=4.0e9)
    tx, rx = _pose_at(0), _pose_at(10.0)
    g_low = ch_low.generate(tx, rx).path_gain_db
    g_high = ch_high.generate(tx, rx).path_gain_db
    assert abs((g_low - g_high) - 6.0) < 0.2, (
        f"Expected ~6 dB difference for 2x freq, got {g_low - g_high}")
    print(f"  [PASS] free_space freq scaling: 2GHz={g_low:.1f}, 4GHz={g_high:.1f} dB")


def test_free_space_doppler():
    """f_d = v * f_c / c for purely radial motion."""
    ch = FreeSpaceChannel(carrier_hz=2.4e9)
    tx = _pose_at(0)
    rx = _pose_at(10.0, vx=10.0)  # moving away at 10 m/s
    st = ch.generate(tx, rx)
    expected = 10.0 * 2.4e9 / 2.99792458e8
    assert abs(st.doppler_hz - expected) < 1.0, (
        f"Expected Doppler {expected:.2f} Hz, got {st.doppler_hz:.2f}")
    print(f"  [PASS] free_space Doppler: {st.doppler_hz:.2f} Hz (expected {expected:.2f})")


def test_free_space_max_range():
    """Beyond max_range_m the link is marked down."""
    radio = RadioParameters(max_range_m=50.0)
    ch = FreeSpaceChannel(carrier_hz=2.4e9, radio=radio)
    st = ch.generate(_pose_at(0), _pose_at(100.0))
    assert not st.is_up, "Link beyond max_range should be down"
    assert math.isinf(st.path_gain_db)
    print(f"  [PASS] free_space max_range cutoff at 50m")


# ---------------------------------------------------------------------------
# Log-distance
# ---------------------------------------------------------------------------

def test_log_distance_basic():
    ch = LogDistanceChannel(carrier_hz=2.0e9, d0_m=1.0, exponent=3.0,
                            shadowing_std=0.0)
    tx = _pose_at(0)
    g1 = ch.generate(tx, _pose_at(1.0)).path_gain_db
    g10 = ch.generate(tx, _pose_at(10.0)).path_gain_db
    # 10x distance, exponent 3 => 30 dB extra loss
    assert abs((g1 - g10) - 30.0) < 0.5, (
        f"Expected ~30 dB for n=3, got {g1 - g10}")
    print(f"  [PASS] log_distance n=3: 1m={g1:.1f}, 10m={g10:.1f} dB")


# ---------------------------------------------------------------------------
# Cellular (3G/4G)
# ---------------------------------------------------------------------------

def test_cellular_band_presets():
    """Each band preset must resolve to a carrier frequency."""
    for band in ("3g_900", "3g_1900", "4g_700", "4g_1800", "4g_2600"):
        ch = CellularChannel(band=band, shadowing_std=0.0)
        assert ch.carrier_hz > 0
        st = ch.generate(_pose_at(0), _pose_at(50.0))
        _assert_state_well_formed(st)
    print(f"  [PASS] cellular bands: 5 presets resolved")


def test_cellular_unknown_band_raises():
    try:
        CellularChannel(band="not_a_band")
        assert False, "Should have raised KeyError"
    except KeyError:
        pass
    print(f"  [PASS] cellular unknown band raises KeyError")


# ---------------------------------------------------------------------------
# 5G NR
# ---------------------------------------------------------------------------

def test_nr_channel():
    ch = NRChannel(carrier_hz=3.5e9, shadowing_std=0.0)
    tx, rx = _pose_at(0), _pose_at(100.0)
    st = ch.generate(tx, rx)
    _assert_state_well_formed(st)
    # CI model: PL = 32.4 + 20*n*log10(d) + 20*log10(f_GHz)
    expected_pl = 32.4 + 20.0 * 2.0 * math.log10(100.0) + 20.0 * math.log10(3.5)
    assert abs(st.path_gain_db - (-expected_pl)) < 0.5, (
        f"Expected PG={-expected_pl:.2f} dB, got {st.path_gain_db:.2f}")
    print(f"  [PASS] NR CI model @100m 3.5GHz: PG={st.path_gain_db:.2f} dB")


# ---------------------------------------------------------------------------
# mmWave
# ---------------------------------------------------------------------------

def test_mmwave_oxygen_absorption():
    """60 GHz should have noticeably more loss than 28 GHz at the same d."""
    ch28 = MmWaveChannel(carrier_hz=28e9, shadowing_std=0.0)
    ch60 = MmWaveChannel(carrier_hz=60e9, shadowing_std=0.0)
    tx, rx = _pose_at(0), _pose_at(500.0)
    g28 = ch28.generate(tx, rx).path_gain_db
    g60 = ch60.generate(tx, rx).path_gain_db
    # 60 GHz has ~15 dB/km O2 absorption vs 0.1 for 28 GHz, plus 20log10 freq term.
    assert g60 < g28, "60 GHz should be worse than 28 GHz at 500 m"
    # O2 difference alone: (15 - 0.1) dB/km * 0.5 km = 7.45 dB
    assert (g28 - g60) > 7.0, (
        f"Expected >7 dB extra loss at 60 GHz, got {g28 - g60:.2f}")
    print(f"  [PASS] mmWave O2: 28GHz={g28:.1f}, 60GHz={g60:.1f} dB @500m")


# ---------------------------------------------------------------------------
# Fiber
# ---------------------------------------------------------------------------

def test_fiber_near_lossless():
    ch = FiberChannel(capacity_bps=10e9)
    tx, rx = _pose_at(0), _pose_at(5.0)
    st = ch.generate(tx, rx)
    _assert_state_well_formed(st)
    # 5 m of fiber at 0.2 dB/km => 0.001 dB loss, essentially zero
    assert st.path_gain_db > -0.01, f"Fiber loss too high: {st.path_gain_db}"
    assert st.capacity_bps == 10e9
    assert st.ber < 1e-9, f"Fiber BER too high: {st.ber}"
    assert st.doppler_hz == 0.0
    print(f"  [PASS] fiber @5m: PG={st.path_gain_db:.4f} dB, cap={st.capacity_bps/1e9:.1f} Gbps")


def test_fiber_cable_length_override():
    ch = FiberChannel(capacity_bps=1e9, cable_length_m=1000.0)
    st = ch.generate(_pose_at(0), _pose_at(1.0))
    # Should use cable_length, not position distance
    assert abs(st.distance_m - 1000.0) < 1e-6
    # 1 km at 0.2 dB/km => 0.2 dB
    assert abs(st.path_gain_db - (-0.2)) < 0.01
    print(f"  [PASS] fiber cable_length override: 1km => {st.path_gain_db:.2f} dB")


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def test_create_channel_factory():
    """create_channel resolves names and aliases."""
    assert isinstance(create_channel("free_space", carrier_hz=2.4e9), FreeSpaceChannel)
    assert isinstance(create_channel("4g", band="4g_2600"), CellularChannel)
    assert isinstance(create_channel("5g", carrier_hz=3.5e9), NRChannel)
    assert isinstance(create_channel("mmwave", carrier_hz=28e9), MmWaveChannel)
    assert isinstance(create_channel("fiber", capacity_bps=1e9), FiberChannel)
    print(f"  [PASS] create_channel factory: all technologies resolved")


def test_list_channels():
    names = list_channels()
    for expected in ("free_space", "cellular", "nr", "mmwave", "fiber"):
        assert expected in names, f"Missing channel '{expected}' in {names}"
    print(f"  [PASS] list_channels: {names}")


# ---------------------------------------------------------------------------
# PHY helpers
# ---------------------------------------------------------------------------

def test_modulations_registry():
    mods = list_modulations()
    for m in ("BPSK", "QPSK", "16QAM", "64QAM", "256QAM"):
        assert m in mods, f"Missing modulation {m}"
    spec = get_modulation("QPSK")
    assert spec.bits_per_symbol == 2
    print(f"  [PASS] modulations: {mods}")


def test_ber_monotonic():
    """BER decreases as SNR increases, for every modulation."""
    for mod in list_modulations():
        ber_low = ber_for(mod, 0.0)
        ber_high = ber_for(mod, 20.0)
        assert ber_low > ber_high, (
            f"{mod}: BER should drop with SNR (low={ber_low}, high={ber_high})")
    print(f"  [PASS] BER monotonic in SNR for all modulations")


def test_thermal_noise():
    """N = kTB.  At 290K, 20 MHz, NF=7dB -> about -100 dBm + NF."""
    radio = RadioParameters(bandwidth_hz=20e6, noise_figure_db=0.0,
                            temperature_k=290.0)
    n_dbm = thermal_noise_dbm(radio)
    # k*T*B at 290K, 20MHz ≈ 8.0e-14 W = -101 dBm
    expected = 10 * math.log10(1.380649e-23 * 290.0 * 20e6 / 1e-3)
    assert abs(n_dbm - expected) < 0.5, f"Expected {expected:.2f}, got {n_dbm:.2f}"
    print(f"  [PASS] thermal noise @20MHz 290K: {n_dbm:.2f} dBm")


def test_shannon_capacity():
    # At 0 dB SNR, 1 Hz BW -> log2(2) = 1 bit/s
    cap = shannon_capacity_bps(0.0, 1.0)
    assert abs(cap - 1.0) < 1e-6, f"Expected 1.0, got {cap}"
    # At -inf SNR -> 0
    assert shannon_capacity_bps(float("-inf"), 1e6) == 0.0
    print(f"  [PASS] Shannon capacity: 0dB/1Hz -> {cap:.4f} bps")


# ---------------------------------------------------------------------------
# THz
# ---------------------------------------------------------------------------

def test_thz_basic():
    """THz channel produces well-formed state with absorption metadata."""
    radio = RadioParameters(bandwidth_hz=10e9, max_range_m=100.0,
                            modulation="16QAM")
    ch = THzChannel(carrier_hz=300e9, radio=radio)
    tx, rx = _pose_at(0), _pose_at(5.0)
    st = ch.generate(tx, rx, tx_id="a", rx_id="b")
    _assert_state_well_formed(st)
    assert st.metadata["model"] == "thz"
    assert st.metadata["absorption_db_per_m"] > 0
    assert st.metadata["absorption_loss_db"] > 0
    print(f"  [PASS] THz @300GHz 5m: pg={st.path_gain_db:.1f} dB, "
          f"abs_loss={st.metadata['absorption_loss_db']:.2f} dB")


def test_thz_absorption_grows_with_distance():
    """Molecular absorption loss must increase with distance."""
    radio = RadioParameters(bandwidth_hz=10e9, max_range_m=100.0,
                            modulation="16QAM")
    ch = THzChannel(carrier_hz=300e9, radio=radio, shadowing_std=0.0)
    tx = _pose_at(0)
    g5 = ch.generate(tx, _pose_at(5.0)).path_gain_db
    g20 = ch.generate(tx, _pose_at(20.0)).path_gain_db
    # Both FSPL and absorption grow, so g20 must be much worse.
    assert g20 < g5
    # Absorption at 20m should be 4x absorption at 5m
    abs5 = ch.generate(tx, _pose_at(5.0)).metadata["absorption_loss_db"]
    abs20 = ch.generate(tx, _pose_at(20.0)).metadata["absorption_loss_db"]
    assert abs(abs20 / abs5 - 4.0) < 0.1, (
        f"Absorption should scale linearly: {abs5:.2f} vs {abs20:.2f}")
    print(f"  [PASS] THz absorption: 5m={abs5:.2f} dB, 20m={abs20:.2f} dB (4x)")


def test_thz_high_freq_more_absorption():
    """350 GHz (water line) should have more absorption than 200 GHz."""
    ch_low = THzChannel(carrier_hz=200e9)
    ch_high = THzChannel(carrier_hz=350e9)
    assert ch_high.absorption_db_per_m > ch_low.absorption_db_per_m
    print(f"  [PASS] THz absorption: 200GHz={ch_low.absorption_db_per_m:.2f} dB/m, "
          f"350GHz={ch_high.absorption_db_per_m:.2f} dB/m")


# ---------------------------------------------------------------------------
# FSO
# ---------------------------------------------------------------------------

def test_fso_basic():
    """FSO channel produces well-formed state with turbulence metadata."""
    ch = FSOChannel()
    tx, rx = _pose_at(0), _pose_at(20.0)
    st = ch.generate(tx, rx, tx_id="a", rx_id="b")
    _assert_state_well_formed(st)
    assert st.metadata["model"] == "fso"
    assert st.metadata["rytov_variance"] >= 0
    assert 0 < st.metadata["pointing_collection"] <= 1
    print(f"  [PASS] FSO @20m 1550nm: pg={st.path_gain_db:.1f} dB, "
          f"rytov={st.metadata['rytov_variance']:.4f}, "
          f"collected={st.metadata['pointing_collection']:.3f}")


def test_fso_pointing_degrades_with_jitter():
    """Increasing jitter must decrease the collection fraction (more loss)."""
    tx, rx = _pose_at(0), _pose_at(20.0)
    pg_low = FSOChannel(pointing_jitter_m=0.001).generate(tx, rx).path_gain_db
    pg_high = FSOChannel(pointing_jitter_m=0.05).generate(tx, rx).path_gain_db
    assert pg_high < pg_low, (
        f"More jitter should mean more loss: low={pg_low}, high={pg_high}")
    print(f"  [PASS] FSO pointing: 1mm={pg_low:.1f} dB, 50mm={pg_high:.1f} dB")


def test_fso_turbulence_grows_with_distance():
    """Rytov variance must increase with distance."""
    ch = FSOChannel(cn2=1e-13)  # strong turbulence for visible effect
    tx = _pose_at(0)
    r5 = ch.generate(tx, _pose_at(5.0)).metadata["rytov_variance"]
    r50 = ch.generate(tx, _pose_at(50.0)).metadata["rytov_variance"]
    assert r50 > r5
    print(f"  [PASS] FSO turbulence: 5m σ_R²={r5:.4f}, 50m σ_R²={r50:.4f}")


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run():
    print("Test: Communication Channel Models (Layer 0+)")
    test_free_space_basic()
    test_free_space_frequency_scaling()
    test_free_space_doppler()
    test_free_space_max_range()
    test_log_distance_basic()
    test_cellular_band_presets()
    test_cellular_unknown_band_raises()
    test_nr_channel()
    test_mmwave_oxygen_absorption()
    test_fiber_near_lossless()
    test_fiber_cable_length_override()
    test_create_channel_factory()
    test_list_channels()
    test_modulations_registry()
    test_ber_monotonic()
    test_thermal_noise()
    test_shannon_capacity()
    test_thz_basic()
    test_thz_absorption_grows_with_distance()
    test_thz_high_freq_more_absorption()
    test_fso_basic()
    test_fso_pointing_degrades_with_jitter()
    test_fso_turbulence_grows_with_distance()
    print()


if __name__ == "__main__":
    run()
