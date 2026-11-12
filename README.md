# Active Suspension 3D

Interactive 3D **suspension lab** of a **1/4-scale single-seater (F2-style)**, built to teach vehicle dynamics. Everything runs in the browser, on desktop, tablet and phone.

**Live:** https://active-susp.vercel.app · first version (front axle only): https://active-susp.vercel.app/v1/

## What you can do

- **Drive** the car: heave, roll, pitch and steering inputs, plus braking, acceleration, cornering, slalom and kerb scenarios. The tyres always stay on the road and every link is solved live.
- **Set up** each axle: camber, toe, caster, kingpin inclination (KPI), scrub radius, mechanical trail, ball joint heights, ride height (push/pullrod length) and track width. The vehicle settings cover wheelbase, centre of gravity height, brake balance and steering ratio.
- **Switch** each axle between **pushrod** (rocker on top of the chassis) and **pullrod** (rocker low in the chassis).
- **Move any pickup point** in 3D with a gizmo, or type its coordinates. The right side mirrors the left.
- **Analyse** the geometry: roll centre heights, camber gain, bump steer, motion ratio, anti-dive / anti-squat, Ackermann, and live curves over wheel travel. The **Geometry** overlay draws the instant centres, roll centres, roll axis and steering axes in 3D.
- **Share** a setup as a link, or export / import it as JSON.

## Active and manual modes

Each rocker is coaxial with a servo. The servo output carries a **drive disc** just behind the rocker hub. A **ball lock pin** through both holes locks the rocker to the servo, like a dog clutch.

| | Active mode | Manual mode |
|---|---|---|
| Ball lock pin | Through rocker hub and drive disc | Parked in a holster |
| Rocker | Locked to the servo | Free on its bearing |
| Servo power | ON | OFF (holster micro-switch pressed) |
| Who moves the suspension | The servos | The user, by hand |
| Kinematics | **Identical** | **Identical** |

Changing mode: the suspension first returns to static ride height so the pins are unloaded. The pins are then parked (manual) or re-inserted once each servo has aligned its disc (active).

## The model

- 1/4 scale of an F2-style car: 784 mm wheelbase, 380 / 365 mm track, 168 mm tyre diameter.
- Double wishbones on all four corners, front steering rack with column and steering wheel, rear toe links and driveshafts.
- Dimensions in mm, ISO axes: **x forward, y left, z up**, origin on the ground at the front axle centre.
- The ball joints are not stored directly: they follow from the alignment settings (KPI, caster, scrub radius, trail and ball joint heights). Moving a ball joint by hand updates those settings back.

## Project structure

```
index.html      Page layout and UI
style.css       Styles (floating panel on desktop, bottom sheet on mobile)
main.js         Three.js scene, UI, scenarios and the active/manual sequencer
suspension.js   3D suspension kinematics and analysis (pure math, no Three.js)
body.js         Lofted F2-style bodywork
v1/             First version (front axle demonstrator)
```

No build step and no dependencies to install: Three.js is loaded from a CDN through an import map.

## Using the kinematics on their own

`suspension.js` has no dependency and runs in Node:

```bash
node -e "import('./suspension.js').then(S => { const car = S.buildCar(S.cloneSetup(S.DEFAULT_SETUP)); console.log(S.solveCar(car, { heave: -10 }).fl) })"
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

- The servo drives the rocker directly, so it must hold the damper load times the rocker arm length. Size the servo (or add a reduction) for the real model mass.
- Support the rocker axle with an outrigger bearing so the servo shaft does not carry the suspension loads.
- Only pull the pin at static ride height, when it carries no load.

## Disclaimer

Concept model for education. The geometry is representative of an F2-style car, not a reproduction of a specific car.
