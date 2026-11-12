# Active Suspension 3D

Interactive 3D model of the **front axle of a 1/4-scale single-seater (F2-style)**, built to teach vehicle dynamics.

**Live demo:** https://active-susp.vercel.app

## Concept

The car uses a classic open-wheel layout: double wishbones (A-arms), an upright, a **pushrod** rising from the upright to a **rocker** mounted on top of the monocoque, and a **coilover** lying across the chassis to a central mount.

Each rocker can be driven by a servo placed **coaxially with the rocker pivot**. The servo output carries a **drive disc** sitting just behind the rocker hub. Rocker hub and drive disc each have a hole at the same radius: a **ball lock pin** inserted through both holes locks the rocker to the servo, like a dog clutch.

| | Active mode | Manual mode |
|---|---|---|
| Ball lock pin | Through rocker hub + drive disc | Parked in a holster on the chassis |
| Rocker | Locked to the servo | Free on its bearing |
| Servo power | ON | OFF (holster micro-switch pressed) |
| Who moves the suspension | The servo turns the rocker | The user pushes the car by hand |
| Kinematics | **Identical**: same rocker, pushrod, wishbones, upright, wheel and coilover motion | **Identical** |

Because the servo acts on the rocker itself, the whole corner moves exactly as it does when the car is pushed by hand. In manual mode nothing drives the servo, so it can neither be damaged nor back-feed current into the electronics.

### Mode change sequence

- **Active → Manual:** the servo returns the corner to static ride height (the pin is then unloaded), the pin is pulled out of the rocker and parked in the holster, where it presses the micro-switch that cuts servo power.
- **Manual → Active:** hands off, the springs bring the corner back to static ride height; the pin leaves the holster (servo power ON), the servo aligns the drive disc hole with the rocker hole, then the pin is inserted.

## Features

- Navigable 3D view (orbit, zoom, pan) with mouse or touch
- Real double wishbone kinematics: wheel travel, rocker angle, coilover length, motion ratio and camber change are solved live
- The body heaves and rolls while the tyres stay on the ground
- Heave and roll inputs (servo command in active mode, hand push in manual mode)
- Active mode scenarios: braking (dive), cornering (roll), kerb strike
- Animated, collision-free pin transfer between rocker and holster
- Camera presets (Iso, Front, Top, Rocker), labels and X-ray toggles
- Responsive layout (desktop, tablet, phone) and on-demand rendering to save battery

## Project structure

```
index.html      Page layout and UI
style.css       Styles (floating panel on desktop, bottom sheet on mobile)
main.js         Three.js scene, mode sequencer and interaction logic
kinematics.js   Pure planar kinematics of one front corner (no Three.js dependency)
```

No build step and no dependencies to install: Three.js is loaded from a CDN through an import map.

## Customising the geometry

All hard points are defined in `HP` at the top of `kinematics.js` (1 unit = 10 mm, x lateral, y vertical, right-hand corner seen from the front): wishbone pivots and ball joints, wheel centre, pushrod pick-up, rocker pivot and arms, coilover chassis mount, pin hole and holster position. The module can be tested on its own with Node:

```bash
node -e "import('./kinematics.js').then(k => console.log(k.solveCorner(1)))"
```

## Run locally

```bash
npx serve .
# or
python3 -m http.server 8000
```

Then open http://localhost:8000 (ES modules do not load from `file://`).

## Deploy

Static folder: on [Vercel](https://vercel.com), import the repository with the **Other** framework preset and no build command.

## Notes for a physical build

- The servo drives the rocker directly, so it must hold the coilover load times the rocker arm length. Size the servo (or add a reduction) for the real model mass.
- Support the rocker axle with an outrigger bearing so the servo shaft does not carry the suspension loads.
- Only pull the pin at static ride height, when it carries no load.

## Disclaimer

Concept model for education. Geometry is representative of an F2-style front axle, not a reproduction of a specific car.

## License

[MIT](LICENSE)
