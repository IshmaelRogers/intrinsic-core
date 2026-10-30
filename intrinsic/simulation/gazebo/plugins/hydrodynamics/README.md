# Hydrodynamics plugin: world gravity rule and evidence

Docs only. This file adds no behavior. The plugin is
`intrinsic::simulation::Hydrodynamics` (#122).

## World rule (D2-A)

A world that runs a Physics system **and** loads
`intrinsic::simulation::Hydrodynamics` must set

```xml
<gravity>0 0 0</gravity>
```

The plugin's wrench already contains the vehicle's weight and buoyancy (the
restoring term). The plugin calls `link.SetGravityEnabled(ecm, false)`, but
bullet-featherstone (the only engine linked here) does not implement the
`GravityEnabled` feature, so gz-sim ignores `GravityEnabledCmd` and the link
keeps engine gravity. With gravity on, weight is applied twice. The calls stay
(they are a no-op under bullet-featherstone and correct under dartsim). The
rule is a world-authoring rule, not an enforced one: a marine world with
default gravity and Hydrodynamics silently sinks.

The plugin is inactive without a `<current_field>` (`kAbsent`, zero wrench).
Use an explicit zero current, `<velocity_m_s>0 0 0</velocity_m_s>`, for no
current.

Checked-in worlds: none runs Physics and loads Hydrodynamics at this commit.
`hydrodynamics_plugin_test.cc` worlds run with no Physics.

## Drag model

Heave, from `six_thruster_uuv_example.cc`: `m dv/dt = F - c_l v - c_q v |v|`
with `m = 32 kg`, `c_l = 35 N s/m` and `c_q = 90 N s^2/m^2` (about 1.1 1/s at
low speed). The default example has displaced volume `1.005 m / rho`, so
`B - W = 0.005 m g = +1.569064 N`. The vehicle is buoyant, with the CoB 4 cm
above the CoG, so it also pitches as a damped hydrostatic oscillator
(`I_yy = 5.4`, `K = 12.599584 N m/rad`, `d_l = 1.5 N m s/rad`).

## Evidence (`imu/imu_plugin_test.cc`, H1 and H2)

Measured with bullet-featherstone, 1 ms step, through the IMU plugin's
differenced acceleration (see `../imu/README.md`).

| Case | World gravity | Measured | Closed form |
| --- | --- | --- | --- |
| H1 `a_w,z`, steps 20-999 | 0 | within 1.2e-5 (every step fresh) | `F_H1 = +1.569064 N`, `a` = 0.04797 at step 20 falling to 0.01492 at step 999 |
| H1 `v_z(1 s)` | 0 | 0.029049 | 0.029040 |
| H1 pitch rate, step 999 | 0 | -3.3127e-3 rad/s | -3.3208e-3 rad/s |
| H2 `a_w,z`, step 1 | -9.80665 | -9.746677 | -9.746683 |
| H2 `a_w,z`, step 50 | -9.80665 | -8.640837 | -8.642778 |

H2 is about -0.995 g, which is the double count. If the engine ever honors
`GravityEnabledCmd`, H2 reads about +0.049 and the test fails, so the
expectations get reviewed.

Known limit: the engine republishes link kinematics only when the link pose
changes by more than 1e-6, so the Hydrodynamics plugin computes drag from
stale velocity at very low speed. This is not changed here.
