# Active Suspension 3D

Interactive 3D model of a **1/4-scale single-seater (F2/F3-style) active suspension**, built for teaching vehicle dynamics.

A servo, coaxial with the rocker, drives the coilover anchor to change ride height and attitude (pitch / roll). A **ball lock pin mode selector** switches the assembly between:

- **Active mode** – the pin links the coilover eye to the servo arm: the servo controls the suspension.
- **Manual mode** – the servo arm returns to neutral, the pin slides back into a fixed chassis bracket and presses a micro-switch that cuts servo power. The suspension becomes purely passive and can be moved by hand without any risk for the servo or the electronics.

**Live demo:** https://active-susp.vercel.app

## Features

- Navigable 3D view (orbit, zoom, pan) with mouse or touch
- Active / Manual mode with sequenced, physically consistent motion (arm returns to neutral before the pin moves)
- Planar kinematics: rocker, pushrod, coilover length and wheel travel are solved live
- Servo angle and wheel travel sliders, demo animation, live readouts
- Camera presets (Iso, Front, Top, Pin), labels and X-ray toggles
- Responsive layout (desktop, tablet, phone) and on-demand rendering to save battery

## Project structure

```
index.html   Page layout and UI
style.css    Styles (floating panel on desktop, bottom sheet on mobile)
main.js      Three.js scene, kinematics and interaction logic
```

No build step and no dependencies to install: Three.js is loaded from a CDN through an import map.

## Run locally

Any static file server works, for example:

```bash
npx serve .
# or
python3 -m http.server 8000
```

Then open http://localhost:8000 (ES modules do not load from `file://`).

## Deploy

The site is a static folder. On [Vercel](https://vercel.com), import the repository with the **Other** framework preset and no build command.

## Customising the model

Geometry is defined by constants at the top of `main.js` (1 scene unit = 10 mm): servo position and arm length, rocker pivot and mount points, pushrod and upright positions, pin positions for each mode. Change them to match your own design.

## Disclaimer

Concept model for education. Proportions are illustrative and not to scale.

## License

[MIT](LICENSE)
